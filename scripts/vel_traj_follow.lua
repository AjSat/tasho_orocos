-- Lua code for following a timed-trajectory of joint velocities with piecewise
-- constant acceleration.

-- Given a sequence of joint position and velocity values, interpolate to
-- to provide a piecewise constant acceleration trajectory to the robot

require("rttlib")
require("math")
require("context")
require("geometric")
require "utils"
require "rttros"

tc=rtt.getTC()
-- the depl will be nil, since for some reason it is not a peer of our component, even if stated explicitly in the deploy script
if tc:getName() == "lua" then
  depl=tc:getPeer("Deployer")
elseif tc:getName() == "Deployer" then
  depl=tc
end

iface_spec = {
   ports={
      { name='joint_vel_out_l_cmsg', datatype='motion_control_msgs/JointVelocities', type='out', desc="joint velocity output as motion control msg" },
      { name='joint_vel_out_arr', datatype='array', type='out', desc="joint velocity output as an array" },
      { name='joint_pos_out', datatype='array', type='out', desc="joint position output" },
      { name='joint_pos_in_actual', datatype='array', type='in', desc="joint position read from robot joint encoder" },
      { name='joint_vel_in_actual', datatype='array', type='in', desc="joint velocity from robot joint encoder" },
      { name='joint_pos_in_ref', datatype='array', type='in', desc="joint position reference from MPC" },
      { name='joint_vel_in_ref', datatype='array', type='in', desc="joint velocity reference from MPC" },
      { name='joint_acc_in_ref', datatype='array', type='in', desc="joint acceleration reference from MPC" },
      { name='event_in', datatype='string', type='in', desc="input events"},
   },

   properties={
      { name='Ts', datatype='double', desc="Sampling time of the trajectory" },
      { name='p_gain', datatype='double', desc="proportional feedback gain on the joint position error" },
      { name='ndof', datatype='int', desc="number of degrees of freedom of the system" },
      { name='no_samples', datatype='int', desc="number of trajectory points sent to the system" },
      { name='joint_pos', datatype='array', desc="joint position" },
      { name='joint_vel', datatype='array', desc="joint velocity" },
      { name='joint_acc', datatype='array', desc="joint acceleration" },
      { name='dt', datatype='double', desc="sample time of the controller" },
   }
}

counter = 0
iface=rttlib.create_if(iface_spec)
iface.props.Ts:set(0.05)
iface.props.ndof:set(14)
iface.props.no_samples:set(0)
iface.props.p_gain:set(0)
iface.props.dt:set(1/250.0)

time_start = 0


function configureHook()

  p_gain= iface.props.p_gain:get()
  Ts= iface.props.Ts:get()
  ndof= iface.props.ndof:get()
  no_samples= iface.props.no_samples:get()
  dt = iface.props.dt:get()

  j_pos_vals_actual = rtt.Variable("array")
  j_vel_vals_actual = rtt.Variable("array")
  j_pos_vals_ref = rtt.Variable("array")
  j_vel_vals_ref = rtt.Variable("array")
  j_vel_vals_command = rtt.Variable("array")
  j_vel_vals_out = rtt.Variable("array")
  j_acc_vals = rtt.Variable("array")
  port_ein = rtt.Variable("string")

  arr_zero = {}
  for i = 1,ndof do
    arr_zero[i] = 0
  end
  -- Initializing joint velocity and acceleration to zero by default
  j_vel_vals_ref:fromtab(arr_zero)
  j_vel_vals_command:fromtab(arr_zero)
  iface.ports.joint_vel_out_arr:write(j_vel_vals_ref)
  j_acc_vals:fromtab(arr_zero)
  iface=rttlib.create_if(iface_spec)

  -- Reading actual joint velocity and joint positions
  fs,j_vel=iface.ports.joint_vel_in_actual:read()
  if fs ~='NoData' then
    j_vel_vals_actual:fromtab(j_vel:totab())
  end
  --
  fs,j_pos=iface.ports.joint_pos_in_actual:read()
  if fs ~='NoData' then
    j_pos_vals_actual:fromtab(j_pos:totab())
    j_pos_vals_ref:fromtab(j_pos:totab()) --Initializing to this assuming that
    -- the reference from MPC also would have the same value.
  end

  return true
end

function startHook()

  -- Updating joint positions and velocities with the latest values
  fs,j_vel=iface.ports.joint_vel_in_actual:read()
  if fs ~='NoData' then
    j_vel_vals_actual:fromtab(j_vel:totab())
  end

  fs,j_pos=iface.ports.joint_pos_in_actual:read()
  if fs ~='NoData' then
    j_pos_vals_actual:fromtab(j_pos:totab())
    j_pos_vals_ref:fromtab(j_pos:totab()) --Initializing to this assuming that
    -- the reference from MPC also would have the same value.
  end
  return true
end

function updateHook()
  -- print("Entering traj_interp updateHook")
  -- Computing the position and velocity references
  fs,j_pos=iface.ports.joint_pos_in_actual:read()
  if fs ~='NoData' then
    j_pos_vals_actual:fromtab(j_pos:totab())
  end

  for i = 0,ndof-1 do
    j_vel_vals_ref[i] = j_vel_vals_ref[i] + j_acc_vals[i]*dt
    j_vel_vals_command[i] = j_vel_vals_ref[i] + p_gain*(j_pos_vals_ref[i] - j_pos_vals_actual[i])
    j_pos_vals_ref[i] = j_pos_vals_ref[i] + j_vel_vals_ref[i]*dt + 0.5*j_acc_vals[i]*dt^2
  end
  -- Reading references from the MPC if available
  fs,j_accr=iface.ports.joint_acc_in_ref:read()
  if fs ~='NoData' and (j_accr[0] ~= j_acc_vals[0] or j_accr[7] ~= j_acc_vals[7]) then
   -- print("Reading acceleration reference")
   j_acc_vals:fromtab(j_accr:totab())

   fs,j_velr=iface.ports.joint_vel_in_ref:read()
   if fs ~='NoData' then
     -- print("Reading velocity reference")
     j_vel_vals_ref:fromtab(j_velr:totab())
     j_vel_vals_command:fromtab(j_velr:totab())
   end
   fs,j_posr=iface.ports.joint_pos_in_ref:read()
   if fs ~='NoData' then
     -- print("Reading position reference")
     j_pos_vals_ref:fromtab(j_posr:totab())
   end

   fs, port_ein=iface.ports.event_in:read()
   if fs ~='NoData' then
     print("OCP finished")
   end



  end

  -- print("position reference:")
  -- print(j_pos_vals_ref)
  -- print("velocity reference")
  -- print(j_vel_vals_ref)
  -- print("acceleration reference")
  -- print(j_acc_vals)

    iface.ports.joint_vel_out_arr:write(j_vel_vals_command) -- TODO add feedback for joint position error

end

function stopHook()
  return true
end
