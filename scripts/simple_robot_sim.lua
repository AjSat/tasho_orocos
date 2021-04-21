-- Lua script for simulating a simple robot

require("rttlib")
require("math")

tc=rtt.getTC()

iface_spec = {
   ports={
      { name='jointpos', datatype='array', type='out', desc="joint positions corresponding to the integrated joint velocities" },
      { name='jointvel_out', datatype='array', type='out', desc="joint velocities output" },
      { name='jointvel', datatype='array', type='in', desc="joint velocities" },
   },

   properties={
      { name='initial_position', datatype='array', desc="initial position of the joints" }
   }
}


iface=rttlib.create_if(iface_spec)

jvals=rtt.Variable("array")
jvel_vals = rtt.Variable("array")

-- The Lua component starts its life in PreOperational, so
-- configureHook can be used to set stuff up.
function configureHook()

    jvals = iface.props.initial_position:get()
    iface.ports.jointpos:write(jvals)

    local temp = {}
    for i = 1, #jvals:totab() do
      temp[i] = 0
    end
    jvel_vals:fromtab(temp)
    iface.ports.jointvel_out:write(jvel_vals)
    return true
end

function startHook()
    configureHook()
    return true
end


function updateHook()
    --print("simple_robot_sim::updateHook")
    fs,vel=iface.ports.jointvel:read()
    if fs~='NoData' then
        local v = vel:totab()
        local dt=tc:getPeriod()
        for i=1,#v do
            -- the arrays are zero-based index, while in LUA it is one-based index.
            -- Markus, you made life difficult and confusing !
            jvals[i-1] = jvals[i-1] + v[i] * dt
        end
      jvel_vals = vel
    end
    iface.ports.jointpos:write(jvals)
    iface.ports.jointvel_out:write(jvel_vals)

end

function stopHook()
  return true
end

function cleanupHook()
    rttlib.tc_cleanup()
end
