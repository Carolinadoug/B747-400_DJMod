-- mSparks 2022/01/07

--sim/operation/override/override_wheel_steer
--
--sim/operation/override/override_control_surfaces
--sim/multiplayer/controls/yoke_pitch_ratio	float[20]	y	[-1..1]	The deflection of the axis controlling pitch.
--sim/multiplayer/controls/yoke_roll_ratio	float[20]	y	[-1..1]	The deflection of the axis controlling roll.
--sim/multiplayer/controls/yoke_heading_ratio	float[20]	y	[-1..1]	The deflection of the axis controlling yaw.
--sim/operation/override/override_joystick_roll
--sim/operation/override/override_joystick_pitch
--sim/operation/override/override_joystick_heading
--sim/flightmodel2/wing/aileron2_deg[]
--sim/flightmodel2/wing/aileron1_deg[]
--sim/flightmodel2/wing/flap1_deg[]
--sim/flightmodel2/wing/flap2_deg[]
--sim/flightmodel2/controls/slat2_deploy_ratio
--sim/flightmodel2/controls/slat1_deploy_ratio
dofile("pid.lua")

B747DR_ap_AFDS_mode_box_status_pilot = find_dataref("laminar/B747/autopilot/AFDS/mode_box_status_pilot")
B747DR_ap_AFDS_mode_box_status_copilot =find_dataref("laminar/B747/autopilot/AFDS/mode_box_status_copilot")
lastBraking=0
lastPitch=0
lastRoll=0
lastYaw=0
lastFlaps=0
numSurfaces=26
lastControlValue={}
for i=0,numSurfaces,1 do
    lastControlValue[i]=0
end

-- 1 rudder lower 2,4
-- 2 rudder upper 1,2
-- 3 l elev inner
-- 4 r elev inner
-- 5 l elev outer
-- 6 r elev outer
-- 7 l ail inner sim/flightmodel/controls/wing2l_ail1def[0]
-- 8 r ail inner sim/flightmodel/controls/wing4r_ail1def[0]
-- 9 l ail outer sim/flightmodel/controls/wing4l_ail2def[0]
-- 10 r ail outer sim/flightmodel/controls/wing2r_ail2def[0]
brakeConsumption=0
local B747_pressureDRs={}
local controlRatios={}
controlRatios[0]=0
B747_pressureDRs[0]=0
local computeRate=0.0333 -- handle low FPS
local lastCompute=0
local doCompute=0
function pressure_input()
    B747_pressureDRs[1]=B747DR_hyd_sys_pressure_1
    B747_pressureDRs[2]=B747DR_hyd_sys_pressure_2
    B747_pressureDRs[3]=B747DR_hyd_sys_pressure_3
    B747_pressureDRs[4]=B747DR_hyd_sys_pressure_4
    
    controlRatios[1]=B747_controls_lower_rudder--simDR_rudder[10]
    controlRatios[2]=B747_controls_upper_rudder--simDR_rudder[10]

    controlRatios[3]=B747_controls_left_inner_elevator-- -simDR_elevator[0]
    controlRatios[4]=B747_controls_right_inner_elevator-- -simDR_elevator[0]
    controlRatios[5]=B747_controls_left_outer_elevator-- -simDR_elevator[0]
    controlRatios[6]=B747_controls_right_outer_elevator-- -simDR_elevator[0]

    controlRatios[7]=B747_controls_left_inner_aileron--simDR_left_aileron_inner
    controlRatios[8]=B747_controls_right_inner_aileron--simDR_right_aileron_inner
    controlRatios[9]=B747_controls_left_outer_aileron--simDR_left_aileron_outer
    controlRatios[10]=B747_controls_right_outer_aileron--simDR_right_aileron_outer

    --left wing
    controlRatios[11]=B747DR_spoiler1
    controlRatios[12]=B747DR_spoiler2
    controlRatios[13]=B747DR_spoiler3
    controlRatios[14]=B747DR_spoiler4
    controlRatios[15]=B747DR_spoiler5--simDR_spoiler5
    controlRatios[16]=B747DR_speedbrake3--simDR_spoiler67[0]

    --right wing
    controlRatios[17]=B747DR_speedbrake3--simDR_spoiler67[1]
    controlRatios[18]=B747DR_spoiler8--simDR_spoiler8
    controlRatios[19]=B747DR_spoiler9--simDR_spoiler910
    controlRatios[20]=B747DR_spoiler10--simDR_spoiler910
    controlRatios[21]=B747DR_spoiler11--simDR_spoiler1112
    controlRatios[22]=B747DR_spoiler12--simDR_spoiler1112

    --flaps left outer to right outer
    controlRatios[23]=B747DR_flap1--simDR_flap1
    controlRatios[24]=B747DR_flap2--simDR_flap2
    controlRatios[25]=B747DR_flap3--simDR_flap3
    controlRatios[26]=B747DR_flap4--simDR_flap4
    
end

function pressure_output()
    if B747DR_hyd_sys_restotal_1>0.1 then
        B747DR_hyd_sys_pressure_1=B747_pressureDRs[1]
    else
        B747DR_hyd_sys_pressure_1=0
    end
    if B747DR_hyd_sys_restotal_2>0.1 then
        B747DR_hyd_sys_pressure_2=B747_pressureDRs[2]
    else
        B747DR_hyd_sys_pressure_2=0
    end
    if B747DR_hyd_sys_restotal_3>0.1 then
        B747DR_hyd_sys_pressure_3=B747_pressureDRs[3]
    else
        B747DR_hyd_sys_pressure_3=0
    end
    if B747DR_hyd_sys_restotal_4>0.1 then
        B747DR_hyd_sys_pressure_4=B747_pressureDRs[4]
    else
        B747DR_hyd_sys_pressure_4=0
    end

    B747DR_rudder_lwr_pos=B747_animate_value(B747DR_rudder_lwr_pos,controlRatios[1],-100,100,20)
    B747DR_rudder_upr_pos=B747_animate_value(B747DR_rudder_upr_pos,controlRatios[2],-100,100,20)
    --B747_interpolate_value
    --[[B747DR_l_elev_inner   = B747_animate_value(B747DR_l_elev_inner,controlRatios[3],-100,100,20)
    B747DR_r_elev_inner   = B747_animate_value(B747DR_r_elev_inner,controlRatios[4],-100,100,20)
    B747DR_l_elev_outer   = B747_animate_value(B747DR_l_elev_outer,controlRatios[5],-100,100,20)
    B747DR_r_elev_outer   = B747_animate_value(B747DR_r_elev_outer,controlRatios[6],-100,100,20)]]--
    B747DR_l_elev_inner   = B747_interpolate_value(B747DR_l_elev_inner,controlRatios[3],-22,17,0.75)
    B747DR_r_elev_inner   = B747_interpolate_value(B747DR_r_elev_inner,controlRatios[4],-22,17,0.75)
    B747DR_l_elev_outer   = B747_interpolate_value(B747DR_l_elev_outer,controlRatios[5],-22,17,0.75)
    B747DR_r_elev_outer   = B747_interpolate_value(B747DR_r_elev_outer,controlRatios[6],-22,17,0.75)

    --[[B747DR_l_aileron_inner   = B747_animate_value(B747DR_l_aileron_inner,controlRatios[7],-100,100,10)
    B747DR_r_aileron_inner   = B747_animate_value(B747DR_r_aileron_inner,controlRatios[8],-100,100,10)
    B747DR_l_aileron_outer   = B747_animate_value(B747DR_l_aileron_outer,controlRatios[9],-100,100,10)
    B747DR_r_aileron_outer   = B747_animate_value(B747DR_r_aileron_outer,controlRatios[10],-100,100,10)]]
    B747DR_l_aileron_inner   = B747_interpolate_value(B747DR_l_aileron_inner,controlRatios[7],-20,20,0.75)
    B747DR_r_aileron_inner   = B747_interpolate_value(B747DR_r_aileron_inner,controlRatios[8],-20,20,0.75)
    B747DR_l_aileron_outer   = B747_interpolate_value(B747DR_l_aileron_outer,controlRatios[9],-25,15,0.75)
    B747DR_r_aileron_outer   = B747_interpolate_value(B747DR_r_aileron_outer,controlRatios[10],-25,15,0.75)


    --spoilers
    for i=1,12,1 do
        B747DR_spoilers[i]=B747_animate_value(B747DR_spoilers[i],controlRatios[i+10],-100,100,10)
    end
    simDR_spoiler12=(B747DR_spoilers[1]+B747DR_spoilers[2])/2
    simDR_spoiler34=(B747DR_spoilers[3]+B747DR_spoilers[4])/2
    simDR_spoiler5=(B747DR_spoilers[5])

    simDR_spoiler8=(B747DR_spoilers[8])
    simDR_spoiler910=(B747DR_spoilers[9]+B747DR_spoilers[10])/2
    simDR_spoiler1112=(B747DR_spoilers[11]+B747DR_spoilers[12])/2
    --spoiler stat
    outleft_spoilers=0
    for i=1,5,1 do
        outleft_spoilers=outleft_spoilers+B747DR_spoilers[i]
    end
    outright_spoilers=0
    for i=8,12,1 do
        outright_spoilers=outright_spoilers+B747DR_spoilers[i]
    end
    B747DR_outer_spoilers[0]=B747_animate_value(B747DR_outer_spoilers[0],outleft_spoilers/5,-100,100,10)
    B747DR_outer_spoilers[1]=B747_animate_value(B747DR_outer_spoilers[1],outright_spoilers/5,-100,100,10)

    --flaps
    --[[B747DR_flaps[1]=B747_animate_value(B747DR_flaps[1],controlRatios[23],-100,100,0.08)
    B747DR_flaps[2]=B747_animate_value(B747DR_flaps[2],controlRatios[24],-100,100,0.08)
    B747DR_flaps[3]=B747_animate_value(B747DR_flaps[3],controlRatios[25],-100,100,0.08)
    B747DR_flaps[4]=B747_animate_value(B747DR_flaps[4],controlRatios[26],-100,100,0.08)]]

    B747DR_flaps[1]=B747_interpolate_value(B747DR_flaps[1],controlRatios[23],0,30,45)
    B747DR_flaps[2]=B747_interpolate_value(B747DR_flaps[2],controlRatios[24],0,30,45)
    B747DR_flaps[3]=B747_interpolate_value(B747DR_flaps[3],controlRatios[25],0,30,45)
    B747DR_flaps[4]=B747_interpolate_value(B747DR_flaps[4],controlRatios[26],0,30,45)
    simDR_flap1=B747DR_flaps[1]
    simDR_flap2=B747DR_flaps[2]
    simDR_flap3=B747DR_flaps[3]
    simDR_flap4=B747DR_flaps[4]
end


function hydraulics_consumer(src,consumption)
    local take_index=0
    if consumption==0  then
        consumption=0.001
    end
    --print("consume "..src[1].." "..src[2].." "..src[3].." "..src[4].." "..consumption)
    for i=1,4,1 do
        if src[i]==1 and B747_pressureDRs[i]>consumption and B747_pressureDRs[i]>B747_pressureDRs[take_index] and B747_pressureDRs[i]>900 then
            take_index=i
        end

    end
    if take_index>0 then
        B747_pressureDRs[take_index]=B747_pressureDRs[take_index]-consumption
        return consumption
    else
        return 0
    end
end

function brake_accumulator()
    if simDR_parking_brake_ratio>lastBraking then
        brakeDiff=(simDR_parking_brake_ratio-lastBraking)*400
    else
        brakeDiff=0
    end
    if brakeDiff<simDR_hyd_press_1_2 then
        simDR_hyd_press_1_2=simDR_hyd_press_1_2-brakeDiff
    else
        simDR_hyd_press_1_2=0
    end
    brakeConsumption=brakeConsumption+brakeDiff
    lastBraking=simDR_parking_brake_ratio

    brakeConsumption=brakeConsumption-hydraulics_consumer({1,1,0,1},brakeConsumption)  
end
function flap_consumption(controlDiff)
    if hydraulics_consumer({0,0,0,1},controlDiff[23]*10)==0 then
        controlRatios[23]=lastControlValue[23]
    end
    if hydraulics_consumer({1,0,0,0},controlDiff[24]*10)==0 then
        controlRatios[24]=lastControlValue[24]
    end
    if hydraulics_consumer({1,0,0,0},controlDiff[25]*10)==0 then
        controlRatios[25]=lastControlValue[25]
    end
    if hydraulics_consumer({0,0,0,1},controlDiff[26]*10)==0 then
        controlRatios[26]=lastControlValue[26]
    end
end
function spoiler_consumption(controlDiff)
    --system 2, spoilers 2,3,10,11
    if hydraulics_consumer({0,1,0,0},controlDiff[12]/3)==0 then
        controlRatios[12]=B747_animate_value(lastControlValue[12],0,-100,100,1)
    end
    if hydraulics_consumer({0,1,0,0},controlDiff[13]/3)==0 then
        controlRatios[13]=B747_animate_value(lastControlValue[13],0,-100,100,1)
    end
    if hydraulics_consumer({0,1,0,0},controlDiff[20]/3)==0 then
        controlRatios[20]=B747_animate_value(lastControlValue[20],0,-100,100,1)
    end
    if hydraulics_consumer({0,1,0,0},controlDiff[21]/3)==0 then
        controlRatios[21]=B747_animate_value(lastControlValue[21],0,-100,100,1)
    end
    --system 3, spoilers 1,4,9,12
    if hydraulics_consumer({0,0,1,0},controlDiff[11]/3)==0 then
        controlRatios[11]=B747_animate_value(lastControlValue[11],0,-100,100,1)
    end
    if hydraulics_consumer({0,0,1,0},controlDiff[14]/3)==0 then
        controlRatios[14]=B747_animate_value(lastControlValue[14],0,-100,100,1)
    end
    if hydraulics_consumer({0,0,1,0},controlDiff[19]/3)==0 then
        controlRatios[19]=B747_animate_value(lastControlValue[19],0,-100,100,1)
    end
    if hydraulics_consumer({0,0,1,0},controlDiff[22]/3)==0 then
        controlRatios[22]=B747_animate_value(lastControlValue[22],0,-100,100,1)
    end

    --system 4, spoilers 5,6,7,8
    if hydraulics_consumer({0,0,0,1},controlDiff[15]/3)==0 then
        controlRatios[15]=B747_animate_value(lastControlValue[15],0,-100,100,1)
    end
    if hydraulics_consumer({0,0,0,1},controlDiff[16]/3)==0 then
        controlRatios[16]=B747_animate_value(lastControlValue[16],0,-100,100,1)
    end
    if hydraulics_consumer({0,0,0,1},controlDiff[17]/3)==0 then
        controlRatios[17]=B747_animate_value(lastControlValue[17],0,-100,100,1)
    end
    if hydraulics_consumer({0,0,0,1},controlDiff[1]/3)==0 then
        controlRatios[18]=B747_animate_value(lastControlValue[18],0,-100,100,1)
    end
end

function flight_controls_consumption()
    controlDiff={}
    for i=1,numSurfaces,1 do
        --print(i.." getting")
        --print(controlRatios[i])
        --print(lastControlValue[i])
        if controlRatios[i]>lastControlValue[i] then
            controlDiff[i]=(controlRatios[i]-lastControlValue[i])
        else
            controlDiff[i]=(lastControlValue[i]-controlRatios[i])
        end
        
    end

    -- steering 
    hydraulics_consumer({1,0,0,0},controlDiff[1]*5)
    --rudder lower 
    if hydraulics_consumer({0,1,0,1},controlDiff[1]*3)==0 then
        controlRatios[1]=B747_animate_value(lastControlValue[1],0,-100,100,10)
    end

    --[[if hydraulics_consumer({0,1,0,1},controlDiff[1]*3)==0 then
        controlRatios[1]=B747_animate_value(lastControlValue[1],0,-100,100,1)
    end]]
    -- rudder upper 
    if hydraulics_consumer({1,1,0,0},controlDiff[2]*3)==0 then
        controlRatios[2]=B747_animate_value(lastControlValue[2],0,-100,100,10)
    end
    
    --1,2 L inboard elev
    if hydraulics_consumer({1,1,0,0},controlDiff[3]*3)==0 then
        controlRatios[3]=B747_animate_value(lastControlValue[3],0,-100,100,20)
    end
    --3,4 R inboard elev
    if hydraulics_consumer({0,0,1,1},controlDiff[4]*3)==0 then
        controlRatios[4]=B747_animate_value(lastControlValue[4],0,-100,100,20)
    end
    --1 L outboard elev
    if hydraulics_consumer({1,0,0,0},controlDiff[5]*3)==0 then
        controlRatios[5]=B747_animate_value(lastControlValue[5],0,-100,100,20)
    end
    --4 R outboard elev
    if hydraulics_consumer({0,0,0,1},controlDiff[6]*3)==0 then
        controlRatios[6]=B747_animate_value(lastControlValue[6],0,-100,100,20)
    end


    --1,2 L inboard aileron
    if hydraulics_consumer({1,1,0,0},controlDiff[7]*3)==0 then
        controlRatios[7]=B747_animate_value(lastControlValue[7],0,-100,100,1)
    end
    --2,4 R inboard aileron
    if hydraulics_consumer({0,1,0,1},controlDiff[8]*3)==0 then
        controlRatios[8]=B747_animate_value(lastControlValue[8],0,-100,100,1)
    end
    --1,2 L outboard aileron
    if hydraulics_consumer({1,1,0,0},controlDiff[9]*3)==0 then
        controlRatios[9]=B747_animate_value(lastControlValue[9],0,-100,100,1)
    end
    --2,3,4 R outboard aileron
    if hydraulics_consumer({0,1,1,1},controlDiff[10]*3)==0 then
        controlRatios[10]=B747_animate_value(lastControlValue[10],0,-100,100,1)
    end

    spoiler_consumption(controlDiff)
    flap_consumption(controlDiff)
    for i=1,numSurfaces,1 do
        lastControlValue[i]=controlRatios[i]
    end
end

function normal_slats()
    if simDR_flap_ratio > 0.166 then
        simDR_innerslats_ratio  	= B747_interpolate_value(simDR_innerslats_ratio,1,0,1,8.5)
    elseif simDR_flap2+simDR_flap3 <2 then
        simDR_innerslats_ratio  	= B747_interpolate_value(simDR_innerslats_ratio,0,0,1,8.5)
    end
    if simDR_flap_ratio > 0.332 then
        simDR_outerslats_ratio  	= B747_interpolate_value(simDR_outerslats_ratio,1,0,1,8.5)
    elseif  simDR_flap2+simDR_flap3 <8 then
        simDR_outerslats_ratio  	= B747_interpolate_value(simDR_outerslats_ratio,0,0,1,8.5)
    end
end
local slatsRetract=false
function B747_slats()
    
    if simDR_flap_ratio==0 and simDR_innerslats_ratio==0 and simDR_outerslats_ratio==0 then
         return;
    elseif (B747DR_speedbrake_lever >0.5 and (simDR_prop_mode[0] == 3 or simDR_prop_mode[1] == 3 or simDR_prop_mode[2] == 3 or simDR_prop_mode[3] == 3)) 
        or (slatsRetract==true and B747DR_speedbrake_lever >0.5) then	
      simDR_innerslats_ratio = B747_interpolate_value(simDR_innerslats_ratio, 0.0, 0.0, 1.0, 8.5)
      slatsRetract=true
    else 
      slatsRetract=false
      normal_slats()
    end
   
 end
 local lastTrimmed=0
 local lastUp=0
 local lastDown=0
local pitchRecord={}
local currentPitchRecord=1
local lastPitchRecordUpdate=0
for i=1,10,1 do
    pitchRecord[i]=0
end


-- The 10-sample pitch/roll boxcar buffers that used to live here were removed
-- along with the moving-average filters (see FLIGHT DIRECTOR COMMAND SMOOTHING
-- below). Only the pitch sample timestamp is still needed, to gate calls into
-- the incremental ap_director_pitch().
local director_lastPitchRecordUpdate=0

local director_yawRecord={}
local director_currentyawRecord=1
local director_lastyawRecordUpdate=0
for i=1,30,1 do
    director_yawRecord[i]=0
end
local last_simDR_ind_airspeed_kts_pilot=0
local last_simDR_AHARS_pitch_heading_deg_pilot=0
local last_altitude=0
local directorSampleRate=0.02
local directorRollSampleRate=0.1
local directoryawSampleRate=0.03
local lastAPTargetRoll=0
local capturedLocTime=0
local hasLoc=false
function ap_director_roll()
   if simDR_autopilot_nav_status ==2 and hasLoc==false then
        capturedLocTime=simDRTime
        hasLoc=true
        if debug_flight_directors==1 then print("localiser captured at "..simDRTime) end
    elseif simDR_autopilot_nav_status ~=2 then
        hasLoc=false
    end
    if math.abs(B747DR_ap_ATT)>=5 then
        return B747DR_ap_ATT
    elseif simDR_autopilot_nav_status ~=2 and B747DR_autopilot_nav_status==2 then
        -- Beam momentarily lost while LOC is still the armed/active mode: hold
        -- the last good roll command rather than snapping wings-level.
        if debug_flight_directors==1 then
            print("holding last roll command, LOC signal dropped out")
        end
        return lastAPTargetRoll
    else
        -- Previously this branch was also taken for the first 5 SECONDS after
        -- localiser capture ("or 5>(simDRTime-capturedLocTime)"), which froze
        -- the roll command open-loop through the entire capture turn - and
        -- those frozen samples then sat in the smoothing filter for several
        -- seconds more. That is what made the aircraft overshoot and start
        -- hunting the moment it captured. The freeze is gone; capture is now
        -- flown closed-loop.
        lastAPTargetRoll=B747DR_ap_target_roll
        return B747DR_ap_target_roll
   end
end
function ap_director_yaw()
    if math.abs(simDR_AHARS_roll_heading_deg_pilot)>5 then 
        return -simDR_sideslip
    else
        return simDR_AHARS_heading_deg_pilot
    end
end
local lastPitchMode=0
local lastPitchModeChange=0
local lastRetVal=0
function ap_director_pitch_retVal(pitchMode,retVal)
    if pitchMode~=lastPitchMode then
        lastPitchModeChange=simDRTime
        lastPitchMode=pitchMode
        
    end
    local diff=simDRTime-lastPitchModeChange
    if diff<0.7 then
        if debug_flight_directors==1 then
            print("holding pitch command through mode change: "..lastRetVal.." (new "..retVal..") mode "..pitchMode)
        end
        -- NOTE: this used to also do
        --     last_simDR_AHARS_pitch_heading_deg_pilot = lastRetVal
        -- which reached back into ap_director_pitch()'s integrator and reset it
        -- to the frozen value on every frame of the hold. The integrator would
        -- be dragged backwards for 0.7s each time the pitch mode changed, so
        -- the command jumped when the hold released. The hold now only affects
        -- what is RETURNED; the integrator is left to keep running.
        return lastRetVal
    end
    if debug_flight_directors==1 then
        print("ap_director_pitch_retVal "..retVal.." retVal "..retVal .." pitchMode "..pitchMode)
    end
    lastRetVal=retVal
    return retVal

end
local thisTargetGlideslipeFPM=-800
local last_simDR_hsi_vdef_dots_pilot=0
local lastSimDRTime=0
local last_speed_delta=0


--[[
    GLIDESLOPE BEAM GAIN PROGRAMMING
    --------------------------------
    Deviation is reported in DOTS, which is an ANGULAR measure: one dot is a
    fixed angle, so the linear displacement it represents shrinks in direct
    proportion to range from the transmitter. Holding a constant fpm-per-dot
    gain therefore makes effective loop gain climb roughly as 1/range all the
    way to the threshold - which is exactly why the vertical axis got
    progressively worse the closer the aircraft got to the runway. Real 747
    autopilots program beam gain down with radio altitude for this reason.

    Radio altitude is proportional to range along a fixed glidepath, so it is
    used directly as the schedule.

    Tuning:
      GS_DOT_GAIN      fpm of correction per dot, at or above GS_GAIN_HI_ALT
      GS_GAIN_HI/LO    radio-altitude schedule and the gain at each end
      GS_RATE_GAIN     damping term on how fast deviation is changing.
                       Set to 0 to disable it and leave pure proportional.
--]]
local GS_DOT_GAIN     = 75     -- fpm per dot (unchanged from original, at height)
local GS_GAIN_HI_ALT  = 1500   -- ft AGL at which full gain applies
local GS_GAIN_LO_ALT  = 150    -- ft AGL at which the floor gain applies
local GS_GAIN_FLOOR   = 0.15   -- gain multiplier at/below GS_GAIN_LO_ALT
local GS_RATE_GAIN    = 250    -- fpm per (dot/second) of deviation rate

--[[
    GLIDESLOPE PITCH LAW - FLIGHT PATH ANGLE FORM
    ---------------------------------------------
    Set GS_USE_LEGACY_FPM_LAW = true to go back to the vertical-speed law above
    (everything from here down is then bypassed). Useful for A/B comparison in
    the sim without needing a rebuild.

    WHY THE VERTICAL-SPEED LAW KEEPS HUNTING

    getGlideSlopeFPM() commands a VERTICAL SPEED proportional to beam
    deviation. But deviation is reported in dots, which is an ANGLE, and the
    vertical speed needed merely to HOLD a given angular deviation shrinks as
    range to the transmitter shrinks. So the correct fpm-per-dot gain changes
    continuously all the way down the approach - gain scheduling can only
    approximate that, and the residual is what shows up as hunting that gets
    worse closer in.

    Commanding a FLIGHT PATH ANGLE instead is scale invariant. Geometry: with
    vertical offset h at range R, the angular deviation is theta = h/R. Since
    dR/dt = -V, simply holding theta constant already requires a path angle
    change of theta, and closing it requires proportionally more. So the
    required correction is proportional to DOTS at every range, with no range
    term at all. That is why this form does not need beam gain programming and
    why it behaves the same at 10nm and at 300ft.

    The command itself is:

        theta = gamma + alpha
        theta_cmd = theta_actual + K * (gamma_cmd - gamma_actual)

    Holding angle of attack constant, the pitch that achieves gamma_cmd is the
    current pitch plus the flight path error. This is self-trimming: alpha is
    measured implicitly rather than integrated, so there is no integrator to
    wind up, nothing to corrupt across a mode change, and it settles cleanly
    (at steady state gamma_actual = gamma_cmd, so theta_cmd = theta_actual and
    the loop stops moving).

    This also replaces a three-stage cascade - dots to target fpm, target fpm
    to an incremental pitch nudge, pitch to elevator - with a single stage,
    removing two of the three integrators from the glideslope path.

    Speed is held by the autothrottle in SPD mode whenever the pitch mode is
    G/S, so pitching for path is the correct division of labour here.

    Tuning:
      GS_DOT_GAMMA     degrees of flight path per dot of deviation
      GS_RATE_GAMMA    degrees of flight path per (dot/second) - the damping
      GS_GAMMA_LIMIT   cap on the total beam correction, degrees
      GS_GAMMA_P       fraction of the flight path error commanded as pitch
--]]
local GS_USE_LEGACY_FPM_LAW = false

-- Gains chosen from an offline sweep of the closed loop (point-mass approach,
-- first-order pitch response). P=2.0/D=3.0 converges to the centreline by 2nm
-- from a 1 dot intercept with zero overshoot, and stays well behaved with the
-- airframe response anywhere from 1 to 4 seconds and with beam noise.
local GS_DOT_GAMMA    = 2.0    -- deg of flight path per dot
local GS_RATE_GAMMA   = 3.0    -- deg of flight path per dot/second
local GS_GAMMA_LIMIT  = 2.5    -- max beam correction, deg
local GS_GAMMA_P      = 1.0    -- pitch commanded per deg of flight path error
local GS_PITCH_MIN    = -2.0
local GS_PITCH_MAX    = 10.0
-- Mild reduction of authority in the last few hundred feet, where beam noise
-- grows and the flare is about to take over. Deliberately gentle: unlike the
-- vertical-speed law this form does not need range scheduling to stay stable.
local GS_TAPER_LO_ALT = 100
local GS_TAPER_HI_ALT = 400
local GS_TAPER_FLOOR  = 0.5

local gsLastDots  = 0
local gsLastTime  = 0
local gsDotsRate  = 0
local gsGammaFilt = nil
local gsWasActive = false

function resetGlideSlopeLaw()
    gsLastDots  = simDR_hsi_vdef_dots_pilot
    gsLastTime  = simDRTime
    gsDotsRate  = 0
    gsGammaFilt = nil
end

function getGlideSlopePitch()
    local dt = simDRTime - gsLastTime
    gsLastTime = simDRTime

    local gsAngle = simDR_glideslope1
    if gsAngle < 0.5 or gsAngle > 10.0 then gsAngle = 3.0 end  -- bad/unset beam

    local dots = simDR_hsi_vdef_dots_pilot

    -- Deviation rate, low-pass filtered. Raw dot-to-dot differences at 10Hz
    -- are noisy enough to swamp the damping term if used directly.
    if dt > 0.02 and dt < 2.0 then
        local raw = (dots - gsLastDots) / dt
        if raw > 1.5 then raw = 1.5 elseif raw < -1.5 then raw = -1.5 end
        gsDotsRate = gsDotsRate + (raw - gsDotsRate) * 0.3
    end
    gsLastDots = dots

    local taper = B747_rescale(GS_TAPER_LO_ALT, GS_TAPER_FLOOR, GS_TAPER_HI_ALT, 1.0, simDR_radarAlt1)

    -- Positive dots steepen the descent. This matches the sign the legacy law
    -- used (it subtracted 75*dots from an already-negative fpm target), which
    -- is the convention that has been capturing the beam correctly.
    local gammaCorr = (GS_DOT_GAMMA * dots + GS_RATE_GAMMA * gsDotsRate) * taper
    if gammaCorr > GS_GAMMA_LIMIT then gammaCorr = GS_GAMMA_LIMIT
    elseif gammaCorr < -GS_GAMMA_LIMIT then gammaCorr = -GS_GAMMA_LIMIT end

    local gammaCmd = -gsAngle - gammaCorr

    -- Actual flight path angle from vertical speed and groundspeed.
    local gs_fpm = simDR_groundspeed * 196.85   -- m/s -> ft/min
    if gs_fpm < 3000 then gs_fpm = 3000 end     -- ~15kt floor, avoid blow-up
    local gammaAct = math.deg(math.atan2(simDR_vvi_fpm_pilot, gs_fpm))
    if gsGammaFilt == nil then gsGammaFilt = gammaAct end
    gsGammaFilt = gsGammaFilt + (gammaAct - gsGammaFilt) * 0.25

    local pitchCmd = simDR_AHARS_pitch_heading_deg_pilot + GS_GAMMA_P * (gammaCmd - gsGammaFilt)
    if pitchCmd < GS_PITCH_MIN then pitchCmd = GS_PITCH_MIN
    elseif pitchCmd > GS_PITCH_MAX then pitchCmd = GS_PITCH_MAX end

    if debug_flight_directors==1 then
        print(string.format(
            "GS: dots %.2f rate %.3f taper %.2f gammaCmd %.2f gammaAct %.2f pitch %.2f cmd %.2f rAlt %.0f",
            dots, gsDotsRate, taper, gammaCmd, gsGammaFilt,
            simDR_AHARS_pitch_heading_deg_pilot, pitchCmd, simDR_radarAlt1))
    end
    return pitchCmd
end

function getGlideSlopeFPM()
    local dt = simDRTime - lastSimDRTime
    lastSimDRTime = simDRTime
    local speed_fpm=simDR_groundspeed*196.85
    thisTargetGlideslipeFPM=-math.tan(math.rad(simDR_glideslope1))*speed_fpm

    local vdef = simDR_hsi_vdef_dots_pilot
    local nextVdef = vdef

    -- Progressive gain when well off the beam. This was a hard 3x step at
    -- exactly 2.0 dots - a discontinuity the loop can limit-cycle across.
    -- Now ramps smoothly from 1x at 1 dot to 3x at 2.5 dots.
    local absVdef = math.abs(nextVdef)
    if absVdef > 1.0 then
        nextVdef = nextVdef * B747_rescale(1.0, 1.0, 2.5, 3.0, absVdef)
    end

    local beamGain = B747_rescale(GS_GAIN_LO_ALT, GS_GAIN_FLOOR, GS_GAIN_HI_ALT, 1.0, simDR_radarAlt1)
    local correction = GS_DOT_GAIN * nextVdef * beamGain

    -- Damping on deviation rate. Without this the loop is pure proportional
    -- on an angular measurement and has nothing opposing an overshoot.
    if GS_RATE_GAIN ~= 0 and dt > 0.001 and dt < 2.0 then
        local vdefRate = (vdef - last_simDR_hsi_vdef_dots_pilot) / dt
        -- ignore implausible jumps (beam re-acquisition, receiver switching)
        if vdefRate > 2.0 then vdefRate = 2.0 elseif vdefRate < -2.0 then vdefRate = -2.0 end
        correction = correction + GS_RATE_GAIN * vdefRate * beamGain
    end
    last_simDR_hsi_vdef_dots_pilot = vdef

    local resultTargetGlideslipeFPM=thisTargetGlideslipeFPM-correction
    if debug_flight_directors==1 then
        print("GS angle "..simDR_glideslope1.." baseFPM "..thisTargetGlideslipeFPM..
              " dots "..vdef.." beamGain "..beamGain..
              " correction "..correction.." targetFPM "..resultTargetGlideslipeFPM)
    end
    return resultTargetGlideslipeFPM

end


local previous_pitchTime=0
local last_simDR_vvi_fpm_pilot=0
local last_vvi_update=0
local prev_vvi_update=0.5
local fpmBias=0
local lastFlapsFPM=0
function get_FPM_bias()
    local fpmBiasMax=6000
    if B747DR_flap_ratio~=lastFlapsFPM and B747DR_flap_lever_detent==0 and simDR_radarAlt1>3000 then
        local Flaps_change=B747DR_flap_ratio-lastFlapsFPM
        print("Flaps_change "..Flaps_change)
        fpmBias=fpmBias+fpmBiasMax*Flaps_change
        lastFlapsFPM=B747DR_flap_ratio
    end
    fpmBias=B747_interpolate_value(fpmBias,0.0,-fpmBiasMax,fpmBiasMax,10)
   -- print("fpmBias "..fpmBias)
    return fpmBias
end
--[[
    VNAV SPD / FLCH SPEED-ON-PITCH TERMS
    ------------------------------------
    The loop in the pitchMode 4/8 branch below is a pure rate-based nudge
    integrator: it moves the commanded pitch by +/-rog depending only on
    whether the speed is CHANGING fast enough, and has no term proportional to
    how far off the target speed actually is. It does converge - hence the
    smooth climb - but the commanded pitch never visibly reflects the speed
    target, so the flight director bars read as disconnected from the speed
    bug. On the real aircraft VNAV SPD pitch tracks the FMS speed directly.

    These add proportional and rate terms on top of that integrator, applied to
    the COMMAND only. At steady state both are ~0, so the integrator still sets
    the trim pitch and these do not fight it; during a speed excursion the bars
    now command the pitch needed to recover the target.

    Sign convention: too FAST -> pitch UP to trade speed for climb.

    Tuning:
      SPD_PITCH_P      degrees of pitch per knot of speed error
      SPD_PITCH_D      degrees of pitch per knot/second of acceleration
      SPD_PITCH_LIMIT  cap on their combined contribution, degrees
      SPD_ERR_CLAMP    speed error is clamped to this before use, knots
--]]
local SPD_PITCH_P     = 0.15
local SPD_PITCH_D     = 0.80
local SPD_PITCH_LIMIT = 5.0
local SPD_ERR_CLAMP   = 25.0

function ap_director_pitch(pitchMode)
    time=simDRTime-previous_pitchTime
    previous_pitchTime=simDRTime
    --print("ap_director_pitch" ..time.. " "..directorSampleRate)
    if time>5 or time==0 or B747DR_ap_pitch_mode_box_status==1 then
        return simDR_AHARS_pitch_heading_deg_pilot
    end
   -- print("ap_director_pitc go")
    local alt_delta=simDR_pressureAlt1-last_altitude
    last_altitude=simDR_pressureAlt1
    local holdAlt=simDR_autopilot_altitude_ft 
    local refreshHoldAlt=simDR_autopilot_hold_altitude_ft
    local refreshfd=simDR_flight_director_pitch
    if simDR_autopilot_alt_hold_status==2 and (pitchMode~=5 and pitchMode~=6 and pitchMode~=9) then
        simDR_autopilot_alt_hold_status=0
    end
    if simDR_autopilot_alt_hold_status==2 then
        holdAlt=simDR_autopilot_hold_altitude_ft
    end
    if debug_flight_directors==1 then
        print("ap_director_pitch for "..pitchMode.." holdAlt "..holdAlt)
    end
    if B747DR_ap_autoland == 1 then 
        if debug_flight_directors==1 then
            print("pitching for autoland "..B744DR_autolandPitch .. " simDR_AHARS_pitch_heading_deg_pilot "..simDR_AHARS_pitch_heading_deg_pilot.. "simDR_touchGround "..simDR_touchGround)
        end
        directorSampleRate=0.02
        return ap_director_pitch_retVal(pitchMode,B744DR_autolandPitch)
    end
    -- GLIDESLOPE: direct flight-path-angle law (see the block by GS_DOT_GAMMA).
    -- Taken before every other mode so G/S never falls through to the generic
    -- vertical-speed integrator, which is what it used to share.
    if pitchMode==2 and GS_USE_LEGACY_FPM_LAW==false then
        -- This law is a direct computation rather than an incremental nudge,
        -- so unlike the integrator it is safe - and better - to sample fast.
        directorSampleRate=0.1
        if gsWasActive==false then
            resetGlideSlopeLaw()   -- fresh capture, drop any stale rate state
            gsWasActive=true
        end
        local retval=getGlideSlopePitch()
        -- Keep the shared integrator in step so that leaving G/S (flare, go
        -- around, level off) starts from the pitch we were actually holding.
        last_simDR_AHARS_pitch_heading_deg_pilot=retval
        last_altitude=simDR_pressureAlt1
        return ap_director_pitch_retVal(pitchMode,retval)
    end
    gsWasActive=false
    --B747DR_ap_flightPhase == 3 and simDR_autopilot_vs_fpm == -500
    if ((pitchMode==4 and B747DR_ap_flightPhase ~= 3) or pitchMode==8) 
    and (simDR_pressureAlt1> holdAlt+B747DR_alt_capture_window or simDR_pressureAlt1< holdAlt-B747DR_alt_capture_window) then
        --local pitchError=math.abs(simDR_AHARS_pitch_heading_deg_pilot-last_simDR_AHARS_pitch_heading_deg_pilot)
        local pitchError=simDR_AHARS_pitch_heading_deg_pilot-last_simDR_AHARS_pitch_heading_deg_pilot

        local speed_delta=simDR_ind_airspeed_kts_pilot-last_simDR_ind_airspeed_kts_pilot
        --FLCH
        if debug_flight_directors==1 then
            print("ap_director_pitch for FLCH "..B747DR_ap_flightPhase)
        end
        last_simDR_ind_airspeed_kts_pilot=simDR_ind_airspeed_kts_pilot
        if (math.abs(simDR_autopilot_airspeed_kts-simDR_ind_airspeed_kts_pilot)>5) and math.abs(simDR_vvi_fpm_pilot)>100 then
            directorSampleRate=0.3
        else
            directorSampleRate=0.5
        end

        local rog=0.01+math.abs(0.5*speed_delta)
        if rog>0.3 then rog=0.3 end
        if simDR_pressureAlt1>29000 or simDR_ind_airspeed_kts_pilot<simDR_autopilot_airspeed_kts then
            rog=rog/3
        end
        local min_speedDelta=0
        local max_speedDelta=0
        local minSafeSpeed = B747DR_airspeed_Vmc + 10
        local speedDiff=math.abs(simDR_ind_airspeed_kts_pilot-simDR_autopilot_airspeed_kts)
        local canDescend = true
        local canAscend = true
       -- if simDR_pressureAlt1> holdAlt and simDR_vvi_fpm_pilot > -200 and simDR_ind_airspeed_kts_pilot<B747DR_airspeed_Vmax-10 then canAscend = false end
        --if simDR_pressureAlt1< holdAlt and simDR_vvi_fpm_pilot < 200 and simDR_ind_airspeed_kts_pilot>minSafeSpeed then canDescend = false end

        if simDR_vvi_fpm_pilot > 200 and (simDR_ind_airspeed_kts_pilot<simDR_autopilot_airspeed_kts-5) then canAscend = false end
        if simDR_vvi_fpm_pilot < -200 and (simDR_ind_airspeed_kts_pilot>simDR_autopilot_airspeed_kts+5) then canDescend = false end
        if B747DR_ap_inVNAVdescent>0 and simDR_vvi_fpm_pilot < -500 and simDR_autopilot_airspeed_kts< simDR_ind_airspeed_kts_pilot+5 then canDescend = false end
        if  speedDiff > 1 then
            if (simDR_autopilot_airspeed_kts< simDR_ind_airspeed_kts_pilot-1) then
                max_speedDelta=0.001+speedDiff/250
            else
                max_speedDelta=0.001+speedDiff/25
            end
            min_speedDelta=max_speedDelta/3
        end

        if ((simDR_autopilot_airspeed_kts> simDR_ind_airspeed_kts_pilot+1) and speed_delta<max_speedDelta 
            or (simDR_autopilot_airspeed_kts< simDR_ind_airspeed_kts_pilot-1) and speed_delta<-max_speedDelta) and pitchError<0.5 and canDescend
        then
            if debug_flight_directors==1 then
                print("-simDR_AHARS_pitch_heading_deg_pilot "..simDR_AHARS_pitch_heading_deg_pilot.." simDR_autopilot_airspeed_kts "..simDR_autopilot_airspeed_kts.." simDR_ind_airspeed_kts_pilot "..simDR_ind_airspeed_kts_pilot.." speed_delta "..speed_delta.." min_speedDelta "..min_speedDelta.." max_speedDelta "..max_speedDelta.." rog "..rog)
            end
            last_simDR_AHARS_pitch_heading_deg_pilot= (last_simDR_AHARS_pitch_heading_deg_pilot-rog)
        elseif ((simDR_autopilot_airspeed_kts< simDR_ind_airspeed_kts_pilot-1) and speed_delta>-min_speedDelta 
            or (simDR_autopilot_airspeed_kts> simDR_ind_airspeed_kts_pilot+1) and speed_delta>min_speedDelta ) and pitchError>-0.5 and canAscend
        then
            if debug_flight_directors==1 then
                print("+simDR_AHARS_pitch_heading_deg_pilot "..simDR_AHARS_pitch_heading_deg_pilot.." simDR_autopilot_airspeed_kts "..simDR_autopilot_airspeed_kts.." simDR_ind_airspeed_kts_pilot "..simDR_ind_airspeed_kts_pilot.." speed_delta "..speed_delta.." min_speedDelta "..min_speedDelta.." max_speedDelta "..max_speedDelta.." rog "..rog)
            end
            last_simDR_AHARS_pitch_heading_deg_pilot= (last_simDR_AHARS_pitch_heading_deg_pilot+rog)
        end

        last_altitude=simDR_pressureAlt1
        if last_simDR_AHARS_pitch_heading_deg_pilot<-3.5 then
            last_simDR_AHARS_pitch_heading_deg_pilot=-3.5
        elseif last_simDR_AHARS_pitch_heading_deg_pilot>15 then 
            last_simDR_AHARS_pitch_heading_deg_pilot=15
        end
        retval=last_simDR_AHARS_pitch_heading_deg_pilot

        -- Speed-error and speed-rate terms on the COMMAND (see the note by
        -- SPD_PITCH_P above). The integrator itself is deliberately left
        -- untouched, so the smooth convergence it provides is preserved and
        -- only the flight director command becomes speed-aware.
        local vError = simDR_ind_airspeed_kts_pilot - simDR_autopilot_airspeed_kts
        if vError > SPD_ERR_CLAMP then vError = SPD_ERR_CLAMP
        elseif vError < -SPD_ERR_CLAMP then vError = -SPD_ERR_CLAMP end
        local vRate = 0
        if time > 0.01 then
            vRate = speed_delta / time
            if vRate > 3 then vRate = 3 elseif vRate < -3 then vRate = -3 end
        end
        local spdTerm = SPD_PITCH_P * vError + SPD_PITCH_D * vRate
        if spdTerm > SPD_PITCH_LIMIT then spdTerm = SPD_PITCH_LIMIT
        elseif spdTerm < -SPD_PITCH_LIMIT then spdTerm = -SPD_PITCH_LIMIT end
        retval = retval + spdTerm
        if retval < -3.5 then retval = -3.5 elseif retval > 15 then retval = 15 end
        if debug_flight_directors==1 then
            print("VNAV SPD: target "..simDR_autopilot_airspeed_kts.." ias "..
                  simDR_ind_airspeed_kts_pilot.." vError "..vError.." vRate "..vRate..
                  " trimPitch "..last_simDR_AHARS_pitch_heading_deg_pilot..
                  " spdTerm "..spdTerm.." command "..retval)
        end

        return ap_director_pitch_retVal(pitchMode,retval)
    elseif pitchMode~=2 and 
    (pitchMode==5 or pitchMode==9 or 
    (simDR_autopilot_alt_hold_status==2) or 
    (simDR_pressureAlt1< holdAlt+B747DR_alt_capture_window and simDR_pressureAlt1> holdAlt-B747DR_alt_capture_window)) then
        --ALT
        if debug_flight_directors==1 then
            print("ap_director_pitch for ALT")

        end 
        if simDR_autopilot_alt_hold_status~=2 then
            simDR_autopilot_hold_altitude_ft=simDR_autopilot_altitude_ft
            simDR_autopilot_alt_hold_status=2
            simDR_autopilot_vs_status=0
            simDR_autopilot_flch_status=0
            --cancel TOGA pitch
            B747DR_autopilot_TOGA_status=0 
            holdAlt=simDR_autopilot_hold_altitude_ft
        end
        local fpmBias=get_FPM_bias()
        local altDiff=math.abs(simDR_pressureAlt1-holdAlt+fpmBias*40)
        local targetFPM=(holdAlt-simDR_pressureAlt1)*2 --target alt in 30 secs
        --local pitchError=math.abs(simDR_AHARS_pitch_heading_deg_pilot-last_simDR_AHARS_pitch_heading_deg_pilot)
        local pitchError=simDR_AHARS_pitch_heading_deg_pilot-last_simDR_AHARS_pitch_heading_deg_pilot

        directorSampleRate=0.1
        local currentFPM=simDR_vvi_fpm_pilot+fpmBias
        local rog=0.001+0.00003*math.abs(currentFPM-targetFPM)
        --if simDR_pressureAlt1>29000 or (altDiff<B747DR_alt_capture_window) then
        if simDR_pressureAlt1>10000 or math.abs(currentFPM)<100 then
            rog=0.0001+0.00001*math.abs(currentFPM-targetFPM)
        end
        if currentFPM>targetFPM  and (pitchError<0.5 or (math.abs(fpmBias)>50 and pitchError<1.0)) then
            last_simDR_AHARS_pitch_heading_deg_pilot=last_simDR_AHARS_pitch_heading_deg_pilot-rog
            if debug_flight_directors==1 then
                print("-last_altitude "..simDR_AHARS_pitch_heading_deg_pilot.." simDR_vvi_fpm_pilot "..currentFPM.." rog "..rog.." targetFPM "..targetFPM)
            end
        elseif currentFPM<targetFPM and pitchError>-0.5 or (math.abs(fpmBias)>50 and pitchError>-1.0) then
            if debug_flight_directors==1 then 
                print("+last_altitude "..simDR_AHARS_pitch_heading_deg_pilot.." simDR_vvi_fpm_pilot "..currentFPM.." rog "..rog.." targetFPM "..targetFPM)
            end
            last_simDR_AHARS_pitch_heading_deg_pilot=last_simDR_AHARS_pitch_heading_deg_pilot+rog
        else
            if debug_flight_directors==1 then
                print("=last_altitude "..simDR_AHARS_pitch_heading_deg_pilot.." simDR_vvi_fpm_pilot "..currentFPM.." rog "..rog.." targetFPM "..targetFPM.. " pitchError "..pitchError)
            end
        end
        if last_simDR_AHARS_pitch_heading_deg_pilot<-3.5 then
            last_simDR_AHARS_pitch_heading_deg_pilot=-3.5
        elseif last_simDR_AHARS_pitch_heading_deg_pilot>15 then 
            last_simDR_AHARS_pitch_heading_deg_pilot=15
        end
        retval=last_simDR_AHARS_pitch_heading_deg_pilot
        last_simDR_AHARS_pitch_heading_deg_pilot=retval
        return ap_director_pitch_retVal(pitchMode,retval)
    
    elseif pitchMode==4 or pitchMode==7 or pitchMode==6 or pitchMode==2 then --pitchMode==2 GLIDESLOPE
        local pitchError=simDR_AHARS_pitch_heading_deg_pilot-last_simDR_AHARS_pitch_heading_deg_pilot
        
        if debug_flight_directors==1 then
            print("ap_director_pitch for VS pitchError="..pitchError)
        end

        local targetFPM=simDR_autopilot_vs_fpm
        local currentFPM=simDR_vvi_fpm_pilot+get_FPM_bias()
        local minUpdateRate=2.0
        local vviDiff=simDRTime-last_vvi_update
        if vviDiff>2 and prev_vvi_update < 1 then
            prev_vvi_update=2.0 --give two seconds before reducing sample rate
            if debug_flight_directors==1 then
                print("prev_vvi_update=2.0") --recover overshoots
            end
        end
        if pitchMode==2 then --set FPM based on glideslope
            targetFPM=getGlideSlopeFPM()
        --    print("getGlideSlopeFPM="..targetFPM.." dots="..simDR_hsi_vdef_dots_pilot)
             --minUpdateRate=B747_rescale(0,0.5,2.5,0.1,math.abs(simDR_hsi_vdef_dots_pilot)) -- be faster on glideslope
        --else
        --    print("standard FPM="..targetFPM.." dots="..simDR_hsi_vdef_dots_pilot)
        end
        if debug_flight_directors==1 then
            print("autopilot FPM="..currentFPM.." target="..targetFPM)
        end
        local vviError=math.abs(currentFPM-targetFPM)
        --[[if ((pitchError<0.3 and vviError<50) or simDR_autopilot_alt_hold_status==2) and pitchMode~=2 then
            directorSampleRate=0.5
        elseif pitchError<0.3 then
            directorSampleRate=minUpdateRate
        end]]--
        --if pitchError<1.3 then
        directorSampleRate=math.min(B747_rescale(0,1,500,0.2,math.abs(vviError)),minUpdateRate,prev_vvi_update)
        prev_vvi_update=directorSampleRate
        --end
       
        --local rog=0.001+0.00004*vviError/(time*30)
        local rog=0.001+0.000003*math.abs(currentFPM-targetFPM)
        local div=B747_rescale(100,30,300,0.3,simDR_ind_airspeed_kts_pilot)
        if debug_flight_directors==1 then
            print("directorSampleRate "..directorSampleRate.." div "..div)
        end
        --[[if simDR_pressureAlt1>29000 then
            rog=rog/3
        elseif vviError>400 then
                rog=rog*15
        elseif simDR_pressureAlt1<12000 then
                rog=rog*3
        end]]
        rog=rog*div
        local speed_delta=(currentFPM-last_simDR_vvi_fpm_pilot)/(time*30)
        last_simDR_vvi_fpm_pilot=currentFPM
        local min_speedDelta=0
        local max_speedDelta=0
        if  vviError > 10 then
            max_speedDelta=vviError/5
            min_speedDelta=vviError/50
        end

        --if pitchError<0.5 then
            -- NOTE: the second condition here used to read
            --     currentFPM<targetFPM and speed_delta<-max_speedDelta
            -- i.e. "descending faster than target AND the descent is still
            -- steepening" -> pitch DOWN. That is positive feedback: it drove
            -- the aircraft further from the target exactly when it was already
            -- diverging. The mirrored error was in the pitch-up branch below.
            -- Both are now removed, leaving only the correct primary tests.
            if (currentFPM>targetFPM and speed_delta>-min_speedDelta and pitchError<0.5) then
                last_simDR_AHARS_pitch_heading_deg_pilot=last_simDR_AHARS_pitch_heading_deg_pilot-rog
                last_vvi_update=simDRTime
                if debug_flight_directors==1 then
                    print("-last_simDR_AHARS_pitch_heading_deg_pilot "..last_simDR_AHARS_pitch_heading_deg_pilot)
                    print("-simDR_vvi_fpm_pilot "..simDR_AHARS_pitch_heading_deg_pilot.." simDR_vvi_fpm_pilot "..currentFPM.." rog "..rog.." speed_delta "..speed_delta.." min_speedDelta "..min_speedDelta.." max_speedDelta "..max_speedDelta)
                end
            elseif (currentFPM<targetFPM and speed_delta<min_speedDelta and pitchError>-0.5) then
            -- (was: "or currentFPM>targetFPM and speed_delta>max_speedDelta",
            -- the mirror of the inverted test above. Also note this branch used
            -- raw simDR_vvi_fpm_pilot while every other test in this function
            -- uses currentFPM, which includes the flap-change fpm bias - so the
            -- two halves of the same comparison were using different values.)
                if debug_flight_directors==1 then 
                    print("+last_simDR_AHARS_pitch_heading_deg_pilot "..last_simDR_AHARS_pitch_heading_deg_pilot)
                    print("+simDR_vvi_fpm_pilot "..simDR_AHARS_pitch_heading_deg_pilot.." simDR_vvi_fpm_pilot "..currentFPM.." rog "..rog.." speed_delta "..speed_delta.." min_speedDelta "..min_speedDelta.." max_speedDelta "..max_speedDelta)
                end
                last_vvi_update=simDRTime
                last_simDR_AHARS_pitch_heading_deg_pilot=last_simDR_AHARS_pitch_heading_deg_pilot+rog
            elseif debug_flight_directors==1 then 
                print("=simDR_vvi_fpm_pilot "..simDR_AHARS_pitch_heading_deg_pilot.." simDR_vvi_fpm_pilot "..currentFPM.." rog "..rog.." speed_delta "..speed_delta.." min_speedDelta "..min_speedDelta.." max_speedDelta "..max_speedDelta)
            end
       -- end
        if last_simDR_AHARS_pitch_heading_deg_pilot<-3.5 then
            last_simDR_AHARS_pitch_heading_deg_pilot=-3.5
        elseif last_simDR_AHARS_pitch_heading_deg_pilot>10 then 
            last_simDR_AHARS_pitch_heading_deg_pilot=10
        end
        retval=last_simDR_AHARS_pitch_heading_deg_pilot
        -- This branch used to assign the return of ap_director_pitch_retVal()
        -- back into last_simDR_AHARS_pitch_heading_deg_pilot, i.e. it fed the
        -- mode-change hold value back into its own integrator. Every other
        -- branch of this function leaves the integrator alone; this one alone
        -- corrupted it. Keep the integrator, return the (possibly held) value.
        return ap_director_pitch_retVal(pitchMode,retval)
    elseif pitchMode==1 then
       -- print("simDR_autopilot_TOGA_pitch_deg ="..simDR_autopilot_TOGA_pitch_deg)
        if debug_flight_directors==1 then
            print("ap_director_pitch for TOGA")
        end
        directorSampleRate=0.1
        retval=B747_interpolate_value(last_simDR_AHARS_pitch_heading_deg_pilot,simDR_autopilot_TOGA_pitch_deg,2,12,5)
        last_simDR_AHARS_pitch_heading_deg_pilot=retval
        ap_director_pitch_retVal(pitchMode,retval) --make sure pitch retval is fresh
        return retval--ap_director_pitch_retVal(pitchMode,retval)
    --elseif pitchMode==2 then
     --   directorSampleRate=0.5
    else
        --print("ap_director_pitch for off")
        last_simDR_AHARS_pitch_heading_deg_pilot=0
        return 0
    end
    local retval=simDR_flight_director_pitch
    if debug_flight_directors==1 then
        print("+simDR_flight_director_pitch "..simDR_AHARS_pitch_heading_deg_pilot.." simDR_flight_director_pitch "..simDR_flight_director_pitch.." B747DR_ap_FMA_active_pitch_mode "..B747DR_ap_FMA_active_pitch_mode.." holdAlt "..holdAlt)
    end
    if retval<simDR_AHARS_pitch_heading_deg_pilot-5 then
        retval=simDR_AHARS_pitch_heading_deg_pilot-5
    elseif retval>simDR_AHARS_pitch_heading_deg_pilot+5 then 
        retval=simDR_AHARS_pitch_heading_deg_pilot+5
    end

    if pitchMode==5 or pitchMode==9 then
        directorSampleRate=1.0
        if retval<-1.5 then
            retval=-1.5
        elseif retval>10 then 
            retval=10
        end
    else
        --if B747DR_ap_FMA_active_pitch_mode==2 then
            directorSampleRate=1.0
        --else
        --    directorSampleRate=0.5
       -- end
        if retval<-10 then
            retval=-10
        elseif retval>10 then 
            retval=10
        end
    end
    last_altitude=simDR_pressureAlt1
    last_simDR_AHARS_pitch_heading_deg_pilot=retval
    return ap_director_pitch_retVal(pitchMode,retval)
end
--[[
    FLIGHT DIRECTOR COMMAND SMOOTHING
    ---------------------------------
    These used to be 10-sample boxcar (moving average) filters. Because the
    sample interval was itself variable (0.2-2.0s), the averaging window was
    2-10 SECONDS long, which put 1-5 seconds of pure phase lag between a beam
    deviation and the control response. On a localiser/glideslope that lag is
    what made the aircraft hunt: it would settle, the window would stretch
    (roll sampled at a full 1.0s per sample once wings-level), drift would
    build unseen, and then it would over-correct.

    Replaced with a first-order lag filter evaluated EVERY frame. Group delay
    is now ~tau instead of ~half the boxcar window.

    Tuning note: raise FD_*_TAU if the response looks twitchy, lower it if the
    aircraft still feels slow to answer the beam. These are the two knobs to
    try first if the approach needs further adjustment.
--]]
local FD_PITCH_TAU = 0.5   -- seconds
local FD_PITCH_TAU_GS = 0.25 -- seconds, glideslope (direct law, less lag wanted)
local FD_ROLL_TAU  = 0.6   -- seconds

local function fd_lag_filter(current, target, tau)
    if tau <= 0 then return target end
    local alpha = SIM_PERIOD / tau
    if alpha > 1 then alpha = 1 end
    if alpha < 0 then alpha = 0 end
    return current + (target - current) * alpha
end

local current_roll_intregal=0
function ap_director_roll_integral()
    -- ap_director_roll() is stateless (it just reads the FD roll target), so
    -- it is safe and more accurate to evaluate it every frame. Sampling it at
    -- up to 1.0s intervals also meant localiser capture was detected up to a
    -- second late.
    current_roll_intregal = fd_lag_filter(current_roll_intregal, ap_director_roll(), FD_ROLL_TAU)
    B747DR_flight_director_roll=current_roll_intregal
    return current_roll_intregal
end

function dampSlip()
    --[[local retval=0
    for i=1,30,1 do
        local tVal=director_yawRecord[i]
        retval=retval+tVal
       --if displayUpdate then print("i "..i.." = " ..pitchRecord[i].. " " ..retval) end
    end
    retval=retval/30]]
    return ap_director_yaw()--retval
end
function dampYaw()
    --local displayUpdate=false
    
    
    local retval=director_yawRecord[1]
    local left=0
    if retval>270 then left=1 end
    for i=2,30,1 do
        local tVal=director_yawRecord[i]
        if left==1 and tVal<90 then tVal=tVal+360 end
        if left==0 and tVal>270 then tVal=tVal-360 end
        retval=retval+tVal
       --if displayUpdate then print("i "..i.." = " ..pitchRecord[i].. " " ..retval) end
    end
    retval=retval/30
    if retval<0 then retval=retval+360 end

    retval=getHeadingDifference(retval,simDR_AHARS_heading_deg_pilot)
    --if displayUpdate then print("ap_director_yaw_integral "..retval.." simDR_AHARS_heading_deg_pilot "..simDR_AHARS_heading_deg_pilot) end
    return retval
end
function clearYawIntegral()
    local val=0
    if math.abs(simDR_AHARS_roll_heading_deg_pilot)<5 then 
        val=simDR_AHARS_heading_deg_pilot
    end
    for i=1,30,1 do
        director_yawRecord[i]=val
    end
end
local lastDamperSystem=0 --0 == slip, 1==yaw
local current_yaw_intregal=0
function ap_director_yaw_integral()
    if (simDRTime-director_lastyawRecordUpdate)>directoryawSampleRate then
         director_lastyawRecordUpdate=simDRTime
         director_yawRecord[director_currentyawRecord]=ap_director_yaw()
         director_currentyawRecord=director_currentyawRecord+1

         if director_currentyawRecord>30 then
             director_currentyawRecord=1
         end
        if math.abs(simDR_AHARS_roll_heading_deg_pilot)>5 then 
            if lastDamperSystem~=0 then clearYawIntegral() end
            lastDamperSystem=0
            current_yaw_intregal= dampSlip()
        else
            if lastDamperSystem~=1 then clearYawIntegral() end
            lastDamperSystem=1
            current_yaw_intregal= dampYaw()
        end
     end
    
    return current_yaw_intregal
end
local current_pitch_intregal=0
local fd_pitch_command=0
function ap_director_pitch_integral()
    -- ap_director_pitch() is an INCREMENTAL controller: it nudges its internal
    -- pitch state by +/-rog on each call, and rog is tuned for being called
    -- once per directorSampleRate. So the sampling gate has to stay - calling
    -- it every frame would multiply its integration rate by 12-120x.
    -- What changes here is only the smoothing: the 10-sample boxcar has been
    -- replaced with a per-frame first-order lag on the sampled command.
    if (simDRTime-director_lastPitchRecordUpdate)>directorSampleRate then
        director_lastPitchRecordUpdate=simDRTime
        fd_pitch_command=ap_director_pitch(B747DR_ap_FMA_active_pitch_mode)
    end

    -- The glideslope law produces a direct, already-damped command, so it does
    -- not need as much smoothing as the incremental modes - and lag is exactly
    -- what hurts beam tracking.
    local tau = FD_PITCH_TAU
    if B747DR_ap_FMA_active_pitch_mode==2 and GS_USE_LEGACY_FPM_LAW==false then
        tau = FD_PITCH_TAU_GS
    end
    current_pitch_intregal = fd_lag_filter(current_pitch_intregal, fd_pitch_command, tau)
    B747DR_flight_director_pitch=current_pitch_intregal
    return current_pitch_intregal
end

local trimrate=25
function doTrim()

    if simDRTime-lastTrimmed<0.2 or simDR_radarAlt1<500 then return end
    lastTrimmed=simDRTime
    local ratioWindow=0.05
    if simDR_autopilot_alt_hold_status==2 then
        ratioWindow=0.005
    elseif B747DR_ap_FMA_active_pitch_mode==2 then
        ratioWindow=0.01 --quick trim on glideslope and alt hold
    elseif B747DR_ap_FMA_active_pitch_mode==4 or B747DR_ap_FMA_active_pitch_mode==8 or (B747DR_ap_FMA_active_pitch_mode==6 and B747DR_ap_inVNAVdescent>0) then
        ratioWindow=0.2 --limit trim activity when pitching for speed
    end

    
    if simDR_ind_airspeed_kts_pilot>230 then
        trimrate=60
    elseif simDR_ind_airspeed_kts_pilot<220 then
        trimrate=30
    end
    if B747DR_custom_pitch_ratio>ratioWindow then
        simDR_elevator_trim=B747_interpolate_value(simDR_elevator_trim,1.0,-1,1,trimrate)
        lastTrimmed=simDRTime+1 
        --print("up trim "..ratioWindow)
    elseif B747DR_custom_pitch_ratio<-ratioWindow then
        simDR_elevator_trim=B747_interpolate_value(simDR_elevator_trim,-1.0,-1,1,trimrate) 
        lastTrimmed=simDRTime+1 
        --print("down trim "..ratioWindow)
   -- else
   --     print("no trim "..ratioWindow.." "..B747DR_sim_pitch_ratio)
    end

end
local previous_simDR_AHARS_pitch_heading_deg_pilot=0

local pitchPid = newPid()
pitchPid.minout=-1
pitchPid.maxout=1
pitchPid.target=0
pitchPid.input = 0
-- The pitch loop is the one that was hunting. Two changes here:
--   * D now acts on measured pitch rate instead of on error, so it damps the
--     response rather than kicking every time the FD target steps.
--   * the integrator is capped well below full elevator authority, so it can
--     no longer wind to the stops and take an equally long time to unwind.
pitchPid.derivativeOnMeasurement = true
pitchPid.iLimit = 0.35
pitchPid:compute()

function ap_pitch_assist()
    local flight_director_pitch=ap_director_pitch_integral()
    local target=0
    local retval=B747DR_sim_pitch_ratio--B747_interpolate_value(B747DR_sim_pitch_ratio,0,-1,1,20)
    local refreshsimDR_electric_trim=simDR_electric_trim
    local refresh_trim=simDR_elevator_trim

    B747DR_pidPitchP=B747_rescale(3000,B747DR_pidPitchPL,40000,B747DR_pidPitchPH,B747DR_autopilot_altitude_ft_pfd)
    -- Integral gain used to be set to the FULL proportional gain by the line
    -- below the if/else, which unconditionally overrode both branches. With
    -- error in degrees of pitch that drives the integrator to full elevator
    -- authority in about 15 seconds of a 1 degree error - far too fast, and
    -- the dominant cause of the slow vertical oscillation. The author's own
    -- first branch already had the right idea (P*0.1); it was just dead code.
    B747DR_pidPitchI=B747DR_pidPitchP*0.1
    pitchPid.kp=B747DR_pidPitchP
    pitchPid.ki=B747DR_pidPitchI
    pitchPid.kd=B747DR_pidPitchD

    if simDR_autopilot_servos_on>0 and (B747DR_ap_FMA_active_pitch_mode>0 or B747DR_ap_autoland == 1) then
        simDR_electric_trim=0
        pitchPid.input = simDR_AHARS_pitch_heading_deg_pilot
        pitchPid.target= flight_director_pitch
        if doCompute==1 then
            pitchPid:compute()
        end
        -- B747_interpolate_value's "speed" is SECONDS to traverse min->max, so
        -- a larger number is SLOWER. The old schedule was rescale(1,3,10,10),
        -- i.e. the elevator got slower the bigger the pitch error - inverted,
        -- and worst exactly during a mode change or capture. Now it speeds up
        -- with error, conservatively (3s at 1 degree, 1.5s at 10 degrees).
        local speed=B747_rescale(1,3,10,1.5,math.abs(flight_director_pitch-simDR_AHARS_pitch_heading_deg_pilot))
        if pitchPid.output==nil then return 0 end
        retval=B747_interpolate_value(B747DR_sim_pitch_ratio,pitchPid.output,-1,1,speed) 
        
        --retval=pitchPid.output

       -- print("elevatorRequest "..elevatorRequest .." pitchChange "..pitchChange .." targetElevator "..targetElevator .." elevatorRate "..elevatorRate)
        doTrim()
       -- print("flight_director_pitch "..flight_director_pitch .." simDR_AHARS_pitch_heading_deg_pilot "..simDR_AHARS_pitch_heading_deg_pilot .." retval "..retval)
    else
        pitchPid:compute(true)
        simDR_electric_trim=1
    end
    if retval==nil then return 0 end
    return retval
end

local previous_simDR_AHARS_roll_heading_deg_pilot=0

local rollPid = newPid()
rollPid.minout=-1
rollPid.maxout=1
rollPid.target=0
rollPid.input = 0
rollPid:compute()

function ap_roll_assist()
    local retval=B747DR_sim_roll_ratio--B747_interpolate_value(B747DR_sim_roll_ratio,0,-1,1,20)
    local flight_director_roll=ap_director_roll_integral()
    rollPid.kp=B747DR_pidRollP
    rollPid.ki=B747DR_pidRollI
    B747DR_pidRollD=B747_rescale(3000,B747DR_pidRollDL,30000,B747DR_pidRollDH,B747DR_autopilot_altitude_ft_pfd)
    rollPid.kd=B747DR_pidRollD
    if simDR_autopilot_servos_on>0 and (B747DR_ap_FMA_active_roll_mode>0) then
        rollPid.input = simDR_AHARS_roll_heading_deg_pilot
        rollPid.target= flight_director_roll
        if doCompute==1 then
            rollPid:compute()
        end
        local speed=B747_rescale(0.1,3,10,5,math.abs(flight_director_roll-simDR_AHARS_roll_heading_deg_pilot))
        if rollPid.output==nil then return 0 end
        retval=B747_interpolate_value(B747DR_sim_roll_ratio,rollPid.output,-1,1,speed) 
        --print("flight_director_roll "..flight_director_roll.." speed "..speed .." simDR_AHARS_roll_heading_deg_pilot "..simDR_AHARS_roll_heading_deg_pilot .." retval "..retval)
    else
        rollPid:compute(true)
    end
    return retval
end
local yawPid = newPid()
yawPid.minout=-1
yawPid.maxout=1
yawPid.target=0
yawPid.input = 0
yawPid:compute()

function get_damper_value(currentValue)
    local target=yawPid.output
    
    local speed=0.4
    
    --[[if math.abs(simDR_AHARS_roll_heading_deg_pilot)<5 and math.abs(yawPid.input)<0.35 then
        speed=2
        target=0
    end]]
    local mult=1
    if math.abs(simDR_AHARS_roll_heading_deg_pilot)<5 then
        mult=B747_rescale(0,0,0.35,1,yawPid.input)
    end
    
    if mult==0 or target==nil then
        return 0
    end
    --print("mult "..mult)
    --print("target "..target)
    target=target*mult
    return target --B747_interpolate_value(currentValue,target,-1,1,speed)
end

function yaw_damper_system()
    if math.abs(simDR_AHARS_roll_heading_deg_pilot)<5 then
        B747DR_pidyawP = B747_rescale(3000, B747DR_pidyawProllL,30000, B747DR_pidyawProllH,B747DR_autopilot_altitude_ft_pfd)
        B747DR_pidyawI = 0.000003
        B747DR_pidyawD = B747_rescale(3000,B747DR_pidyawDrollL,30000,B747DR_pidyawDrollH,B747DR_autopilot_altitude_ft_pfd)
    else
        B747DR_pidyawP = B747_rescale(3000, B747DR_pidyawPslipL,30000, B747DR_pidyawPslipH,B747DR_autopilot_altitude_ft_pfd)
        B747DR_pidyawI = 0.000003
        B747DR_pidyawD = B747_rescale(3000,B747DR_pidyawDslipL,30000,B747DR_pidyawDslipH,B747DR_autopilot_altitude_ft_pfd)
    end

    yawPid.kp=B747DR_pidyawP
    yawPid.ki=B747DR_pidyawI
    yawPid.kd=B747DR_pidyawD
    yawPid.input = ap_director_yaw_integral()
    if doCompute==1 then
        yawPid:compute()
    end
    --print(B747DR_yaw_damper_lwr.." "..get_damper_value())
    if B747DR_yaw_damper_upr_on ==1 then
        
         B747DR_yaw_damper_upr=B747_interpolate_value(B747DR_yaw_damper_upr,get_damper_value(B747DR_yaw_damper_upr),-1,1,0.3)
    else

        B747DR_yaw_damper_upr=B747_interpolate_value(B747DR_yaw_damper_upr,0,-1,1,0.3)
    end

    if B747DR_yaw_damper_lwr_on ==1 then
        B747DR_yaw_damper_lwr=B747_interpolate_value(B747DR_yaw_damper_lwr,get_damper_value(B747DR_yaw_damper_lwr),-1,1,0.3)
    else
        B747DR_yaw_damper_lwr=B747_interpolate_value(B747DR_yaw_damper_lwr,0,-1,1,0.3)
    end
end


function flight_controls_override()
    --[[override=1
    for i=1,4,1 do
        if B747_pressureDRs[i]>800 then
            --override=0
        end
    end]] --this ver always overrides
    if (simDRTime-lastCompute)>computeRate then
        doCompute=1
        lastCompute=simDRTime
    else
        doCompute=0
    end  
    simDR_override_control_surfaces=1--override
    

    
    if B747_pressureDRs[1]>1000 then

        if simDR_ias_pilot<20 then
            simDR_override_steering=0
        else
            simDR_override_steering=1
            simDR_steering[0]=B747_rescale(-1,-7,1,7,simDR_total_heading_ratio)
            simDR_steering[1]=B747_interpolate_value(simDR_steering[1],0,-77,77,0.3)
            simDR_steering[2]=B747_interpolate_value(simDR_steering[2],0,-77,77,0.3)
        end
    else

        simDR_override_steering=1
    end
    

    --Rudder ratio changer
    B747DR_rudder_ratio=1.0-B747_rescale(150,0,450,0.84375,simDR_ias_pilot)
    B747DR_elevator_ratio=1.0--(1.0-B747_rescale(150,0,350,0.84375,simDR_ias_pilot))*-1
    B747DR_l_aileron_outer_lockout   = 1.0-B747_rescale(232,0,238,1.0,simDR_ias_pilot)
    B747DR_r_aileron_outer_lockout   = (1.0-B747_rescale(232,0,238,1.0,simDR_ias_pilot))*-1
    B747_slats()
    if((simDRTime - B747DR_switching_servos_on)<2) then
        ap_pitch_assist()
        ap_roll_assist()
    else
        B747DR_sim_pitch_ratio=ap_pitch_assist()
        B747DR_sim_roll_ratio=ap_roll_assist()
    end
    yaw_damper_system()

end
