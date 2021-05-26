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
samplefreq = 100
simulation = false

-- ros:import("rtt_rospack")
depl:import("rtt_ros")
ros = gs:provides("ros")

-- Loading ROS based libraries
ros:import("rtt_rospack")
depl:import("rtt_std_msgs")

ros:import("yumi_tasho")
depl:import("yumi_tasho")
depl:import("rtt_sensor_msgs")
depl:import("rtt_motion_control_msgs")

depl:loadComponent("ocp", "OCPComponent")
ocp = depl:getPeer("ocp")
dir = ros:find("yumi_tasho")
ocp:getProperty("js_prop_file"):set(dir .. "/casadi_files/homing_ocp_fun_property.json")
ocp:configure()
ros:import("etasl_iohandler_jointstate")
--Configuration
--6511 is ROB_L 6512 is ROB_R
home_pos = rtt.Variable("array")
home_pos:fromtab({ -0.19690, -2.33, 1.95, 0.6580, 0.2390, 0.3770, -0.4250, 0, -2.26, -2.35, 0.52, 0.025, 0.749, 0,})
ocp:getProperty("goal_des"):set(home_pos)
arr = rtt.Variable("array")
arr:fromtab({50/180.0*3.14159,})
ocp:getProperty("max_vel"):set(arr)
arr2 = rtt.Variable("array")
arr2:fromtab({120/180*3.14159,})
ocp:getProperty("max_acc"):set(arr2)



-- ocp:setPeriod(0.1)
cp = rtt.Variable("ConnPolicy")
-- cp.type=1   -- type buffered
-- cp.size=5  -- buffer size

depl:loadComponent("traj_interp_left", "OCL::LuaComponent")
traj_interp_left = depl:getPeer("traj_interp_left")
traj_interp_left:exec_file(dir .. "/scripts/vel_traj_follow.lua")
traj_interp_left:getProperty("dt"):set(1/samplefreq)
-- Creating fast controller and simulator for the right robot arm
depl:loadComponent("traj_interp_right", "OCL::LuaComponent")
traj_interp_right = depl:getPeer("traj_interp_right")
traj_interp_right:exec_file(dir .. "/scripts/vel_traj_follow.lua")
traj_interp_right:getProperty("dt"):set(1/samplefreq)

if simulation then

  depl:loadComponent("robot_sim_left", "OCL::LuaComponent")
  robot_sim_left = depl:getPeer("robot_sim_left")
  robot_sim_left:exec_file(dir .. "/scripts/simple_robot_sim.lua")

  -- robot_sim_left:setPeriod(0.004)
  j_init_left = rtt.Variable("array")
  j_init_left:fromtab({-1.36542319, -0.74822507, 2.05658987, 0.52732208, 2.4950726,
   -0.93756902, -1.71694542})
  robot_sim_left:getProperty("initial_position"):set(j_init_left)
  -- robot_sim_right:setPeriod(0.004)

  -- depl:connectPeers("robot_sim_left","traj_interp_left")
  -- depl:connectPeers("robot_sim_left","ocp")
  -- depl:connectPeers("robot_sim_right","traj_interp_right")
  -- depl:connectPeers("robot_sim_right","ocp")
  depl:connect("robot_sim_left.jointpos", "ocp.q_actual_left", cp)
  depl:connect("traj_interp_left.joint_pos_in_actual", "robot_sim_left.jointpos", cp)
  depl:stream("robot_sim_left.jointpos", ros:topic("/joint_states_left_from_orocos"))
  depl:connect("traj_interp_left.joint_vel_out_arr", "robot_sim_left.jointvel", cp)
  depl:setActivity("robot_sim_left", 1/samplefreq, 99, rtt.globals.ORO_SCHED_RT)

  robot_sim_left:configure()
  robot_sim_left:start()
  depl:loadComponent("robot_sim_right", "OCL::LuaComponent")

  -- robot_sim_right = depl:getPeer("robot_sim_right")
  -- robot_sim_right:exec_file(dir .. "/scripts/simple_robot_sim.lua")
  -- j_init_right = rtt.Variable("array")
  -- j_init_right:fromtab({1.32087, -0.77865726, -2.04601662, 0.65292945,
  -- -2.25832585, -0.81930464, 1.00047389})
  -- robot_sim_right:getProperty("initial_position"):set(j_init_right)
  -- depl:connect("traj_interp_right.joint_vel_out_arr", "robot_sim_right.jointvel", cp)
  -- depl:connect("robot_sim_right.jointpos", "ocp.q_actual_right", cp)
  -- depl:connect("traj_interp_right.joint_pos_in_actual", "robot_sim_right.jointpos", cp)
  -- depl:setActivity("robot_sim_right", 1/samplefreq, 99, rtt.globals.ORO_SCHED_RT)
  -- depl:stream("robot_sim_right.jointpos", ros:topic("/joint_states_right_from_orocos"))
  -- robot_sim_right:configure()
  -- robot_sim_right:start()

else
  depl:import("abb_egm_driver")
	--Loading component for the left arm
	depl:loadComponent("yumi_l", "EGM::EGMDriver")
	yumi_l = depl:getPeer("yumi_l")
	--Loading component for the right arm
	depl:loadComponent("yumi_r", "EGM::EGMDriver")
	yumi_r = depl:getPeer("yumi_r")


	--Configuration
	--6511 is ROB_L 6512 is ROB_R
	yumi_l:getProperty("simulation"):set(false)
	yumi_l:getProperty("egm_ip"):set("192.168.125.1")
	yumi_l:getProperty("egm_port"):set(6511)
	depl:setActivity("yumi_l", 0, 99, rtt.globals.ORO_SCHED_RT)
	-- yumi_l:setPeriod(0.004)

	yumi_r:getProperty("simulation"):set(false)
	yumi_r:getProperty("egm_ip"):set("192.168.125.1")
	yumi_r:getProperty("egm_port"):set(6512)
	depl:setActivity("yumi_r", 0, 99, rtt.globals.ORO_SCHED_RT)
	-- yumi_r:setPeriod(0.004)

	-- depl:connectPeers("traj_interp_left", "yumi_l")
  -- depl:connectPeers("ocp", "yumi_l")
	-- depl:connectPeers("traj_interp_right", "yumi_r")
  -- depl:connectPeers("ocp", "yumi_r")

	depl:connect("yumi_l.q_actual", "traj_interp_left.joint_pos_in_actual", cp)
  depl:connect("yumi_l.q_actual", "ocp.q_actual_left", cp)
	depl:connect("yumi_r.q_actual", "traj_interp_right.joint_pos_in_actual", cp)
  depl:connect("yumi_r.q_actual", "ocp.q_actual_right", cp)
	depl:connect("yumi_l.JointVelocityCommand", "traj_interp_left.joint_vel_out_cmsg", cp)
	depl:connect("yumi_r.JointVelocityCommand", "traj_interp_right.joint_vel_out_cmsg", cp)
end

-- Connecting the peers

-- depl:connectPeers("traj_interp_left","ocp")
-- depl:connectPeers("traj_interp_right","ocp")


-- Connecting the ports between the components
depl:connect("traj_interp_left.event_in", "ocp.event_out", cp)
depl:connect("traj_interp_left.joint_pos_in_ref", "ocp.q_command_left", cp)
depl:connect("traj_interp_left.joint_vel_in_ref", "ocp.qdot_command_left", cp)
depl:connect("traj_interp_left.joint_acc_in_ref", "ocp.qddot_command_left", cp)
depl:connect("traj_interp_right.event_in", "ocp.event_out", cp)
depl:connect("traj_interp_right.joint_pos_in_ref", "ocp.q_command_right", cp)
depl:connect("traj_interp_right.joint_vel_in_ref", "ocp.qdot_command_right", cp)
depl:connect("traj_interp_right.joint_acc_in_ref", "ocp.qddot_command_right", cp)

depl:setActivity("ocp", 0.1, 99, rtt.globals.ORO_SCHED_RT)
depl:setActivity("traj_interp_left", 1/samplefreq, 50, rtt.globals.ORO_SCHED_RT)
depl:setActivity("traj_interp_right", 1/samplefreq, 50, rtt.globals.ORO_SCHED_RT)


-- ros:import("fbsched")
-- depl:loadComponent("fbs", "FBSched")
-- fbs=depl:getPeer("fbs")
-- sched_order=fbs:getProperty("sched_order")
-- depl:connect("traj_interp_left.triggerPort","fbs.trigger", cp)
-- depl:setMasterSlaveActivity("fbs", "traj_interp_left")
-- depl:setMasterSlaveActivity("fbs", "traj_interp_right")
-- depl:setMasterSlaveActivity("fbs", "robot_sim_left")
-- depl:setMasterSlaveActivity("fbs", "robot_sim_right")
-- depl:connectPeers("traj_interp_left","fbs")
-- depl:connectPeers("traj_interp_right","fbs")
-- depl:connectPeers("robot_sim_left","fbs")
-- depl:connectPeers("robot_sim_right","fbs")
-- -- depl:connectPeers("timed_traj","fbs")
-- sched_order:get():resize(4)
-- sched_order[0]="traj_interp_left"
-- sched_order[1]="traj_interp_right"
-- sched_order[2]="robot_sim_left"
-- sched_order[3]="robot_sim_right"
-- -- sched_order[1]="timed_traj"
-- -- fbs:start()

--configure hook of both components
traj_interp_left:configure()
traj_interp_left:start()
-- --configure hook of both components
traj_interp_right:configure()
traj_interp_right:start()

if not simulation then
	yumi_l:configure()
	yumi_l:start()
  yumi_r:configure()
	yumi_r:start()
end

ocp:start()
sleep(12.0)
-- --
ocp:stop()
if simulation then
	robot_sim_left:stop()
  robot_sim_right:stop()
else
	yumi_l:stop()
	yumi_r:stop()
end

traj_interp_left:stop()
traj_interp_right:stop()
ocp:cleanup()

-- Reporter:stop()
print("Code finished successfully")
