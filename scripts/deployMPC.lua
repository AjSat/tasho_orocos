-- An illustrative file to test and show the deployment of MPC component

function sleep(n)
  os.execute("sleep " .. tonumber(n))
end

require("rttlib")
require("rttros")
require "utils"
rtt.setLogLevel("Debug")
rttlib.color = true

tc=rtt.getTC()
depl=tc:getPeer("Deployer")
samplefreq = 160
mpc_freq = 20
simulation = true

depl:import("rtt_rospack")
depl:import("rtt_ros")
depl:import("rtt_sensor_msgs")
depl:import("rtt_motion_control_msgs")

-- Loading ROS based libraries

gs = rtt.provides()
ros = gs:provides("ros")
depl:import('rtt_rosnode')
ros:import("rtt_rospack")
depl:import("tasho_orocos")
depl:import("rtt_sensor_msgs")
depl:import("rtt_motion_control_msgs")


depl:loadComponent("mpc", "MPCComponent")
mpc = depl:getPeer("mpc")


--Configuration
--6511 is ROB_L 6512 is ROB_R
dir = rtt.provides("ros"):find("tasho_orocos")
mpc:getProperty("js_prop_file"):set(dir .. "/casadi_files/mpc_fun.json")
mpc:configure()

depl:setActivity("mpc", 1/mpc_freq, 99, rtt.globals.ORO_SCHED_RT)
depl:setPeriod(1/mpc_freq)
cp = rtt.Variable("ConnPolicy")
-- cp.type=1   -- type buffered
-- cp.size=1  -- buffer size

depl:loadComponent("traj_interp_left", "OCL::LuaComponent")
traj_interp_left = depl:getPeer("traj_interp_left")
traj_interp_left:exec_file(dir .. "/scripts/vel_traj_follow.lua")
traj_interp_left:getProperty("Ts"):set(1/mpc_freq)
traj_interp_left:getProperty("dt"):set(1/samplefreq)
traj_interp_left:getProperty("ndof"):set(7)
depl:setActivity("traj_interp_left", 1/samplefreq, 0, rtt.globals.ORO_SCHED_OTHER)

-- Creating a controller for the right arm
depl:loadComponent("traj_interp_right", "OCL::LuaComponent")
traj_interp_right = depl:getPeer("traj_interp_right")
traj_interp_right:exec_file(dir .. "/scripts/vel_traj_follow.lua")
traj_interp_left:getProperty("Ts"):set(1/mpc_freq)
traj_interp_right:getProperty("dt"):set(1/samplefreq)
traj_interp_right:getProperty("ndof"):set(7)
depl:setActivity("traj_interp_right", 1/samplefreq, 0, rtt.globals.ORO_SCHED_OTHER)


if simulation then
  depl:loadComponent("robot_sim_left", "OCL::LuaComponent")
  robot_sim_left = depl:getPeer("robot_sim_left")
  robot_sim_left:exec_file(dir .. "/scripts/simple_robot_sim.lua")
  depl:setActivity("robot_sim_left", 1/samplefreq, 0, rtt.globals.ORO_SCHED_OTHER)
  j_init_left = rtt.Variable("array")
  j_init_left:fromtab({-1.36542319, -0.74822507, 2.05658987, 0.52732208, 2.4950726,
  -0.93756902, -1.71694542})
  robot_sim_left:getProperty("initial_position"):set(j_init_left)
  -- Connecting the ports between the components
  depl:connect("traj_interp_left.joint_vel_out_arr", "robot_sim_left.jointvel", cp)
  depl:connect("robot_sim_left.jointpos", "mpc.q_actual_left", cp)
  -- depl:connect("robot_sim_left.jointpos", "traj_interp_left.joint_pos_in_actual", cp)
  depl:connect("robot_sim_left.jointvel_out", "mpc.qdot_actual_left", cp)
  ros:import("etasl_iohandler_jointstate")
  depl:stream("robot_sim_left.jointpos", ros:topic("/joint_states_left_from_orocos"))
  robot_sim_left:configure()
  robot_sim_left:start()

  j_init_right = rtt.Variable("array")
  j_init_right:fromtab({1.32087, -0.77865726, -2.04601662, 0.65292945,
  -2.25832585, -0.81930464, 1.00047389})
  depl:loadComponent("robot_sim_right", "OCL::LuaComponent")
  robot_sim_right = depl:getPeer("robot_sim_right")
  robot_sim_right:exec_file(dir .. "/scripts/simple_robot_sim.lua")
  depl:setActivity("robot_sim_right", 1/samplefreq, 0, rtt.globals.ORO_SCHED_OTHER)
  robot_sim_right:getProperty("initial_position"):set(j_init_right)
  -- Connecting the ports between the components
  depl:connect("traj_interp_right.joint_vel_out_arr", "robot_sim_right.jointvel", cp)
  depl:connect("robot_sim_right.jointpos", "mpc.q_actual_right", cp)
  -- depl:connect("robot_sim_right.jointpos", "traj_interp_right.joint_pos_in_actual", cp)
  depl:connect("robot_sim_right.jointvel_out", "mpc.qdot_actual_right", cp)
  ros:import("etasl_iohandler_jointstate")
  depl:stream("robot_sim_right.jointpos", ros:topic("/joint_states_right_from_orocos"))
  robot_sim_right:configure()
  robot_sim_right:start()

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
  depl:setActivity("yumi_l", 0, 99, rtt.globals.ORO_SCHED_OTHER)
  yumi_r:getProperty("simulation"):set(false)
  yumi_r:getProperty("egm_ip"):set("192.168.125.1")
  yumi_r:getProperty("egm_port"):set(6512)
  depl:setActivity("yumi_r", 0, 99, rtt.globals.ORO_SCHED_OTHER)

  depl:connect("yumi_l.q_actual", "traj_interp_left.joint_pos_in_actual", cp)
  depl:connect("yumi_l.q_actual", "mpc.q_actual_left", cp)
  depl:connect("yumi_l.q_dot_actual", "mpc.qdot_actual_left", cp)
  depl:connect("yumi_r.q_actual", "traj_interp_right.joint_pos_in_actual", cp)
  depl:connect("yumi_r.q_actual", "mpc.q_actual_right", cp)
  depl:connect("yumi_r.q_dot_actual", "mpc.qdot_actual_right", cp)
  depl:connect("yumi_l.JointVelocityCommand", "traj_interp_left.joint_vel_out_cmsg", cp)
  depl:connect("yumi_r.JointVelocityCommand", "traj_interp_right.joint_vel_out_cmsg", cp)
end

depl:connect("traj_interp_right.joint_pos_in_ref", "mpc.q_command_right", cp)
depl:connect("traj_interp_right.joint_vel_in_ref", "mpc.qdot_command_right", cp)
depl:connect("traj_interp_right.joint_acc_in_ref", "mpc.qddot_command_right", cp)
depl:connect("traj_interp_left.joint_pos_in_ref", "mpc.q_command_left", cp)
depl:connect("traj_interp_left.joint_vel_in_ref", "mpc.qdot_command_left", cp)
depl:connect("traj_interp_left.joint_acc_in_ref", "mpc.qddot_command_left", cp)

traj_interp_left:configure()
traj_interp_left:start()
traj_interp_right:configure()
traj_interp_right:start()

mpc:start()
sleep(5.0)
-- --
mpc:stop()
if simulation then
  robot_sim_left:stop()
  robot_sim_right:stop()
end
traj_interp_left:stop()
traj_interp_right:stop()
mpc:cleanup()

-- Reporter:stop()
print("Code finished successfully")
