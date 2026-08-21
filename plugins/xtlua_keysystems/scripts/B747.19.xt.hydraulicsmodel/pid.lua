-- adaption of https://github.com/OpenPrograms/Kubuxu-Programs/blob/master/pid/pid.lua
-- Mark Parker, April 2022

local pid = {}

local function clamp(x, min, max)
  if x > max then
    return max
  elseif x < min then
    return min
  else
    return x
  end
end
-- Removed: a global sleep() that shelled out via os.execute. It was dead test
-- scaffolding, but it left an arbitrary-shell-command primitive reachable from
-- every script sharing this Lua state.
--local seconds = os.clock

-- all values of the PID controller
-- values with '_' at beginning are considered private and should not be changed.
pid = {
    kp = 0.02,
    ki = 0.003,
    kd = 0.0002,
    input = nil,
    target = nil,
    output = nil,
    minout = -math.huge,
    maxout = math.huge,
    -- derivativeOnMeasurement: when true, the D term acts on the rate of
    -- change of the MEASUREMENT rather than of the error. Derivative-on-error
    -- spikes every time the target moves, and these targets are staircases
    -- coming out of the flight director, so the D term was injecting a kick on
    -- every step instead of damping anything.
    -- Left false by default so existing tuning of the roll and yaw loops is
    -- untouched; enabled explicitly on the pitch loop.
    derivativeOnMeasurement = false,
    -- iLimit: optional cap on the integral term, separate from the output
    -- clamp. Without it the integrator is only limited by full control
    -- authority, so it can wind all the way to the stops and then take just as
    -- long to unwind - a slow oscillation the pilot sees as hunting.
    iLimit = nil,
    _lasttime = nil,
    _lastinput = nil,
    _lasterr= 0,
    _Iterm = 0
  }

  function pid:new(save)
    assert(save == nil or type(save) == "table", "If save is specified the it has to be table.")
    
    save = save or {}
    setmetatable(save, self)
    self.__index = self
    return save
  end
-- Exports calibration variables and targeted value.
function pid:save()
    return {kp = self.kp, ki = self.ki, kd = self.kd, target = self.target, minout = self.minout, maxout = self.maxout}
end 
  -- This is main method of PID controller.
  -- After creation of controller you have to set 'target' value in controller table
  -- then in loop you should regularly update 'input' value in controller table,
  -- call c:compute() and set 'output' value to the execution system.
  -- c.minout = 0
  -- c.maxout = 100
  -- while true do
  --   c.input = getCurrentEnergyLevel()
  --   c:compute()
  --   reactorcontrol:setAllControlRods(100 - c.output) -- PID expects the increase of output value will cause increase of input
  --   sleep(0.5)
  -- end
  -- You can limit output range by specifying 'minout' and 'maxout' values in controller table.
  -- By passing 'true' to the 'compute' function you will cause controller to not to take any actions but only
  -- refresh internal variables. It is most useful if PID controller was disconnected from the system.
  function pid:compute(waspaused)
    assert(self.input and self.target, "You have to sepecify current input and target before running compute()")
    -- reset values if PID was paused for prolonegd period of time
    if waspaused or self._lasttime == nil or self._lastinput == nil then
      self._lasttime = simDRTime--seconds()
      self._lastinput = self.input
      self._Iterm = self.output or 0
      return
    end
    local err = self.target - self.input
    local dtime = simDRTime - self._lasttime
    if dtime <= 0 then
      return
    end
    -- simDRTime is sim/time/total_running_time_sec, which keeps advancing
    -- while the sim is paused, loading or hitching. An unclamped dtime after
    -- one of those produces a huge integral jump and a meaningless derivative.
    if dtime > 0.2 then
      dtime = 0.2
    end

    local iTerm = self._Iterm + self.ki * err * dtime

    local dinput
    if self.derivativeOnMeasurement then
      -- standard form: D acts on the measurement, and enters negatively
      dinput = -(self.input - self._lastinput) / dtime
    else
      dinput = (err - self._lasterr) / dtime
    end

    local unclamped = self.kp * err + iTerm + self.kd * dinput
    self.output = clamp(unclamped, self.minout, self.maxout)

    -- Anti-windup. Only accept the new integral if it is within its own limit
    -- AND the output is not saturated in the direction the integral is
    -- pushing. Otherwise hold the previous value, so the integrator does not
    -- keep charging against the stops and then take just as long to unwind.
    local iMax = self.iLimit or self.maxout
    local iMin = self.iLimit and -self.iLimit or self.minout
    if iTerm > iMax then
      iTerm = iMax
    elseif iTerm < iMin then
      iTerm = iMin
    end
    local saturated = (unclamped >= self.maxout and err > 0) or
                      (unclamped <= self.minout and err < 0)
    if not saturated then
      self._Iterm = iTerm
    end

    self._lasttime = simDRTime--seconds()
    self._lastinput = self.input
    self._lasterr = err
end

function newPid()
  return pid:new()
end
--[[local yaw=0.5


function getCurrentEnergyLevel()
    return yaw
end
function doYaw(rudder)
  if(rudder==nil) then
    return
  end
    yaw=yaw+rudder+rudder*rudder
end
print("hello")
c = pid:new()
c.minout=-1
c.maxout=1
c.target=10
c.input =getCurrentEnergyLevel()
c:compute()
targetTime=0.1
while true do
    c.input =getCurrentEnergyLevel()
    c:compute()

    doYaw(c.output)
    sleep(0.02)
    print(yaw)
    if seconds()>targetTime then 
      targetTime=targetTime+0.1  
      c.target=c.target-5
    end
    
end
print("hello")]]--
