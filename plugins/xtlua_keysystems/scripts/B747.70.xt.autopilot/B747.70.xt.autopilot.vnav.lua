--[[
*****************************************************************************************
* Program Script Name	:	B747.70.autopilot.vnav
* Author Name			:	Mark Parker (mSparks)
*
*   Revisions:
*   -- DATE --	--- REV NO ---		--- DESCRIPTION ---
*   2021-01-27	0.01a				Start of Dev
*
*
*
*
--]]
dofile("B747.70.xt.autopilot.vnavspd.lua")

--[[
    DESCENT: TRADING PATH AGAINST SPEED
    -----------------------------------
    This used to return a flat -500 fpm whenever indicated airspeed was more
    than 5kt above the applicable restriction, and the profile rate otherwise.
    Two things went wrong with that.

    1. -500 was not just a vertical speed, it was also a MODE SIGNAL.
       fma_PitchModes() tested "simDR_autopilot_vs_fpm == -500" to decide
       between VNAV SPD (pitch mode 4) and VNAV PATH (pitch mode 6), and those
       route to two completely different pitch laws - speed-on-pitch versus the
       vertical-speed integrator. So a 5kt threshold with no hysteresis was
       switching the entire vertical control law back and forth, each switch
       also triggering the 0.7s command freeze in ap_director_pitch_retVal().
       That alone is enough to produce sustained hunting.

    2. It coupled "high" and "fast" into positive feedback.
       -500 is far shallower than a descent profile normally needs, so every
       excursion into speed priority put the aircraft further above path. Being
       above path then requires a steeper descent to recover, which makes the
       aircraft faster, which re-triggers speed priority. High and fast
       reinforce each other, which is exactly the reported behaviour: well
       above the vertical target AND not holding the selected speed.

    The rewrite fixes both:

      * The vertical speed is now a smooth blend from the profile rate to the
        deceleration rate as speed excess grows, instead of a step.
      * Willingness to shallow out for speed is scaled DOWN by how far above
        profile the aircraft already is, so the loop above cannot run away.
        At DES_PATH_TOL above profile the path wins outright.
      * The FMA mode signal is now an explicit flag with hysteresis
        (B747_descentSpeedPriority) rather than an exact-equality test on a
        floating point vertical speed.

    When the aircraft genuinely cannot hold both, the autothrottle goes to idle
    and the existing DRAG REQUIRED annunciation in B747_ap_EICAS_msg() tells
    the crew to use speedbrake - which is what the real aircraft does.

    Tuning:
      DES_DECEL_VS    fpm used when decelerating is the priority
      DES_SPD_ENTER   kt above the restriction at which the trade is complete
      DES_SPD_EXIT    kt above the restriction at which speed priority drops
      DES_PATH_TOL    ft above profile at which the path wins outright
--]]
local DES_DECEL_VS  = -500
local DES_SPD_ENTER = 10
local DES_SPD_EXIT  = 3
local DES_PATH_TOL  = 400

-- Read by fma_PitchModes() to choose VNAV SPD vs VNAV PATH. Global on purpose:
-- vnav.lua is dofile'd into the autopilot script, so they share an environment.
B747_descentSpeedPriority = 0

local function descentBlend(targetvspeed, spdExcess)
    -- Ramp from the profile rate to the deceleration rate with speed excess,
    -- rather than stepping between them.
    local want = B747_rescale(0, targetvspeed, DES_SPD_ENTER, DES_DECEL_VS, spdExcess)
    -- Give the path back its authority the further above profile we are.
    local pathPriority = B747_rescale(0, 0.0, DES_PATH_TOL, 1.0, B747BR_fpe)
    return want + (targetvspeed - want) * pathPriority
end

local function setSpeedPriority(spdExcess)
    if B747_descentSpeedPriority == 0 then
        if spdExcess > DES_SPD_ENTER and B747BR_fpe < DES_PATH_TOL * 0.5 then
            B747_descentSpeedPriority = 1
        end
    else
        if spdExcess < DES_SPD_EXIT or B747BR_fpe > DES_PATH_TOL then
            B747_descentSpeedPriority = 0
        end
    end
end

function deceleratedDesent(targetvspeed)
  if simDR_autopilot_airspeed_is_mach == 1 then
    B747_descentSpeedPriority = 0
    return targetvspeed
  end --can't do this in mach mode, slow tf down already

  local meet = B747_rescale(0,0,400,500,B747BR_fpe)

  -- Nil guards: these come from FMC entries that may not be set yet, and a nil
  -- reaching math.max() here would throw inside after_physics every frame.
  local transAlt = tonumber(getFMSData("desspdtransalt"))
  local restAlt  = tonumber(getFMSData("desrestalt"))
  local transSpd = tonumber(getFMSData("destranspd"))
  local restSpd  = tonumber(getFMSData("desrestspd"))
  if transAlt==nil or restAlt==nil or transSpd==nil or restSpd==nil then
    B747_descentSpeedPriority = 0
    return targetvspeed - meet
  end

  local upperAlt, lowerAlt = transAlt, restAlt
  local upperSpd, lowerSpd = transSpd, restSpd
  if restAlt > transAlt then
    upperAlt, lowerAlt = restAlt, transAlt
    upperSpd, lowerSpd = restSpd, transSpd
  end

  -- The profile catch-up term applies on every path, not just when no speed
  -- restriction is nearby.
  local profileVS = targetvspeed - meet

  -- NOTE: 1000ft is under a minute of anticipation at a normal descent rate,
  -- which is not enough for a 747 at idle to shed 40-50kt. Widening it was
  -- tried and rejected: it only buys deceleration by flying shallower than the
  -- profile, i.e. by drifting ABOVE the path, which is the symptom being fixed
  -- here. The real fix is a deceleration segment in the profile itself so the
  -- path is shallower where speed has to come off - see CHANGELOG known issues.
  if simDR_pressureAlt1 > upperAlt + 1000 then
    B747_descentSpeedPriority = 0
    return profileVS
  end --nowhere near a restriction yet

  if simDR_ind_airspeed_kts_pilot <= (lowerSpd + DES_SPD_EXIT) then
    B747_descentSpeedPriority = 0
    return profileVS
  end --already low enough for everything below

  -- Which restriction actually applies at this altitude.
  local limitSpd = upperSpd
  if simDR_pressureAlt1 <= lowerAlt + 1000 then
    limitSpd = lowerSpd
  elseif simDR_ind_airspeed_kts_pilot <= (upperSpd + DES_SPD_EXIT) then
    -- inside the upper restriction and already slow enough for it
    B747_descentSpeedPriority = 0
    return profileVS
  end

  local spdExcess = simDR_ind_airspeed_kts_pilot - limitSpd
  setSpeedPriority(spdExcess)
  return descentBlend(profileVS, spdExcess)
end
function setDescentVSpeed(fmsO)

  if B747BR_totalDistance>=15 then
    --set in setDistances when < 15
    local glideAlt= B747DR_fmstargetDistance*290 +B747DR_ap_vnav_target_alt
    if string.len(B747BR_vnavProfile)>2 then
      local vnavData=json.decode(B747BR_vnavProfile)
      --print("B747BR_vnavProfile in setDescentVSpeed="..B747BR_vnavProfile)
      local endI = table.getn(vnavData)
      for i = 1, endI, 1 do
        if vnavData[i][4] then
          --if i==1 then break end
          --local altDiff=vnavData[i-1][3]-vnavData[i][3]

          local legDist = getDistance(simDR_latitude, simDR_longitude, vnavData[i][1], vnavData[i][2])
          glideAlt= legDist*vnavData[i][5] +vnavData[i][3]
          B747DR_ap_vnav_target_alt=vnavData[i][3]
          B747DR_fmstargetDistance=legDist
          --print("setDescentVSpeed "..glideAlt)
          break
        end
      end
    end
    B747BR_fpe	= simDR_pressureAlt1-glideAlt
  end
  if simDR_autopilot_altitude_ft+600 > simDR_pressureAlt1 then return end --dont set fpm near hold alt  
  local distanceNM=B747DR_fmstargetDistance
  
  
  if distanceNM<1 then
    distanceNM=1
  end

  local nextDistanceInFeet=distanceNM*6076.12
  local time=distanceNM*30.8666/(simDR_groundspeed) --time in minutes, gs in m/s....
  local early=100
  if B747DR_ap_vnav_target_alt>simDR_pressureAlt1 then
    early=0
  elseif simDR_autopilot_altitude_ft>5000 then
    early=250 
  else
    early=B747BR_fpe
  end
  local vdiff=B747DR_ap_vnav_target_alt-simDR_pressureAlt1-early --to be negative
  local vspeed=vdiff/time
  --[[if B747DR_ap_vnav_target_alt>simDR_pressureAlt1 then
    vspeed=0
  end]]--
  --print("setDescentVSpeed speed=".. simDR_groundspeed .. " distance=".. distanceNM .. " vspeed=" .. vspeed .. " vdiff=" .. vdiff .. " time=" .. time.. " B747DR_ap_vnav_target_alt=" .. B747DR_ap_vnav_target_alt)
		  --speed=89.32039642334 distance=2.9459299767094vspeed=-6559410.6729958
  B747DR_ap_vb = math.atan2(vdiff,nextDistanceInFeet)*-57.2958
  if vspeed<-2500 then vspeed=-2500 end
  if vspeed>1500 then vspeed=1500 end
  if simDR_radarAlt1<=10 then
    simDR_autopilot_vs_fpm = -250 -- slow descent, reduces AoA which if it goes to high spoils the landing
    B747DR_ap_inVNAVdescent=0
    B747DR_ap_vnav_state=0
    B747DR_ap_thrust_mode=0
    setDescent(false)
    B747_descentSpeedPriority = 0
    print("End Descent")
    return
  end
  if B747DR_ap_vnav_state > 0 then
    simDR_autopilot_vs_fpm = deceleratedDesent(vspeed)
  else
    -- Nothing is driving the trade any more; do not leave the FMA latched in
    -- VNAV SPD from the last descent.
    B747_descentSpeedPriority = 0
  end
  B747DR_ap_fpa=math.atan2(vspeed,simDR_groundspeed*196.85)*-57.2958
  
  --[[if B747DR_descentSpeedGradient>0 and simDR_pressureAlt1>B747DR_target_descentAlt then
    simDR_autopilot_airspeed_kts=B747DR_target_descentSpeed+(simDR_pressureAlt1-B747DR_target_descentAlt)*B747DR_descentSpeedGradient
    if simDR_autopilot_airspeed_is_mach == 1 then
      B747DR_ap_ias_dial_value=simDR_autopilot_airspeed_kts_mach*100
    end
    --print("set descentSpeed to " .. simDR_autopilot_airspeed_kts)
  end]]

  
end
  
  function getDescentTarget()
    B747DR_target_descentSpeed=tonumber(getFMSData("destranspd"))
    B747DR_target_descentAlt=tonumber(getFMSData("desspdtransalt"))
    if B747DR_target_descentAlt>simDR_pressureAlt1 
      or simDR_autopilot_airspeed_kts<B747DR_target_descentSpeed 
      then 
      B747DR_descentSpeedGradient=0 
      return 
    end
    B747DR_descentSpeedGradient=(simDR_autopilot_airspeed_kts-B747DR_target_descentSpeed)/(simDR_pressureAlt1-B747DR_target_descentAlt)
    print("set descentSpeedGradient to " .. B747DR_descentSpeedGradient)
  end
  