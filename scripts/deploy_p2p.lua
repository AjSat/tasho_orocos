-- Brings Yumi to the  home position. Run after use so that calibration next time
-- is easy.
-- home_pos   = {0, -2.26, -2.35, 0.52, 0.025, 0.749, 0, -0.19690, -2.33, 1.95, 0.6580, 0.2390, 0.3770, -0.4250}

function sleep(n)
  os.execute("sleep " .. tonumber(n))
end


require("rttlib")
require("rttros")
require "utils"
rtt.setLogLevel("Debug")
rttlib.color = true
gs = rtt.provides()
tc=rtt.getTC()
depl=tc:getPeer("Deployer")
samplefreq = 250
move_left_arm = true

-- ros:import("rtt_rospack")
depl:import("rtt_ros")
ros = gs:provides("ros")

-- Loading ROS based libraries
ros:import("rtt_rospack")
depl:import("rtt_std_msgs")

ros:import("tasho_orocos")
depl:import("tasho_orocos")
dir = ros:find("tasho_orocos")
depl:import("rtt_sensor_msgs")
depl:import("rtt_motion_control_msgs")

depl:loadComponent("ocp", "OCPComponent")
ocp = depl:getPeer("ocp")
ocp:getProperty("js_prop_file"):set(dir .. "/casadi_files/leftp2p_ocp_fun2_property.json")
ocp:configure()
ros:import("etasl_iohandler_jointstate")
--Configuration
--6511 is ROB_L 6512 is ROB_R
fk_des = rtt.Variable("array")
-- fk_des:fromtab({ 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.4, 0.4, 0.4}) --left
fk_des:fromtab({ 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.4, 0.4, 0.4}) --left
ocp:getProperty("goal_des"):set(fk_des)
arr = rtt.Variable("array")
arr:fromtab({50/180.0*3.14159,})
ocp:getProperty("max_vel"):set(arr)
arr2 = rtt.Variable("array")
arr2:fromtab({120/180*3.14159,})
ocp:getProperty("max_acc"):set(arr2)

depl:setActivity("ocp", 0, 99, rtt.globals.ORO_SCHED_RT)
ocp:setPeriod(0.1)
cp = rtt.Variable("ConnPolicy")
-- cp.type=1   -- type buffered
-- cp.size=1  -- buffer size

depl:loadComponent("traj_interp", "OCL::LuaComponent")
traj_interp = depl:getPeer("traj_interp")
traj_interp:exec_file(dir .. "/scripts/vel_traj_follow.lua")
traj_interp:getProperty("p_gain"):set(0.1)
depl:setActivity("traj_interp", 0, 99, rtt.globals.ORO_SCHED_RT)
traj_interp:setPeriod(0.004)

depl:loadComponent("robot_sim", "OCL::LuaComponent")
robot_sim = depl:getPeer("robot_sim")
robot_sim:exec_file(dir .. "/scripts/simple_robot_sim.lua")
depl:setActivity("robot_sim", 0, 99, rtt.globals.ORO_SCHED_RT)
robot_sim:setPeriod(0.004)
j_init = rtt.Variable("array")
-- j_init:fromtab({-1.36542319, -0.74822507, 2.05658987, 0.52732208, 2.4950726,
--  -0.93756902, -1.71694542, 1.32087, -0.77865726, -2.04601662, 0.65292945,
--  -2.25832585, -0.81930464, 1.00047389})
j_init:fromtab({-1.36542319, -0.74822507, 2.05658987, 0.52732208, 2.4950726,
 -0.93756902, -1.71694542})
robot_sim:getProperty("initial_position"):set(j_init)

-- Connecting the peers
depl:connectPeers("robot_sim","traj_interp")
depl:connectPeers("robot_sim","ocp")
depl:connectPeers("traj_interp","ocp")



-- Connecting the ports between the components
depl:connect("traj_interp.joint_vel_out_arr", "robot_sim.jointvel", cp)
depl:connect("traj_interp.joint_pos_in_actual", "robot_sim.jointpos", cp)
depl:connect("robot_sim.jointpos", "ocp.q_actual", cp)
-- depl:connect("robot_sim.jointvel_out", "ocp.qdot_actual", cp)
depl:connect("traj_interp.joint_pos_in_ref", "ocp.q_command", cp)
depl:connect("traj_interp.event_in", "ocp.event_out", cp)
depl:connect("traj_interp.joint_vel_in_ref", "ocp.qdot_command", cp)
depl:connect("traj_interp.joint_acc_in_ref", "ocp.qddot_command", cp)
depl:stream("robot_sim.jointpos", ros:topic("/joint_states_left_from_orocos"))
-- depl:stream("ocp.q_command", ros:topic("/joint_states_from_orocos"))
robot_sim:configure()
robot_sim:start()
--configure hook of both components
traj_interp:configure()
traj_interp:start()
for i = 1,1 do
sleep(0.1);
ocp:start()
sleep(6.0)
-- --
ocp:stop()
end
robot_sim:stop()
traj_interp:stop()
ocp:cleanup()

-- Reporter:stop()
print("Code finished successfully")
