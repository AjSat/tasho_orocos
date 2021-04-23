-- An illustrative file to test and show the deployment of MPC component

function sleep(n)
  os.execute("sleep " .. tonumber(n))
end

require("rttlib")
require("rttros")
require "utils"
rtt.setLogLevel("Info")
rttlib.color = true

tc=rtt.getTC()
depl=tc:getPeer("Deployer")
samplefreq = 250

depl:import("rtt_rospack")
depl:import("rtt_ros")
depl:import("rtt_sensor_msgs")
depl:import("rtt_motion_control_msgs")

-- Loading ROS based libraries

gs = rtt.provides()
ros = gs:provides("ros")
depl:import('rtt_rosnode')
ros:import("fbsched")
ros:import("rtt_rospack")
depl:import("yumi_tasho")
depl:import("rtt_sensor_msgs")
depl:import("rtt_motion_control_msgs")
--

-- deploy scheduler (just one component right now):
--depl:loadComponent("fbs", "FBSched")
--fbs=depl:getPeer("fbs")

depl:loadComponent("mpc", "MPCComponent")
mpc = depl:getPeer("mpc")


--Configuration
--6511 is ROB_L 6512 is ROB_R
mpc:getProperty("mpc_rate"):set(20) -- in Hz
mpc:getProperty("horizon"):set(13) -- in Hz
dir = rtt.provides("ros"):find("yumi_tasho")
mpc:getProperty("ocp_file"):set(dir .. "/casadi_files/ocp_fun.casadi")
mpc:getProperty("mpc_file"):set(dir .. "/casadi_files/mpc_fun.casadi")
mpc:getProperty("predict_file"):set(dir .. "/casadi_files/jac_fun_rob.casadi")
mpc:getProperty("shift_file"):set(dir .. "/casadi_files/jac_fun_rob.casadi")

depl:setActivity("mpc", 0, 99, rtt.globals.ORO_SCHED_RT)
mpc:setPeriod(0.05)
cp = rtt.Variable("ConnPolicy")
-- cp.type=1   -- type buffered
-- cp.size=1  -- buffer size

depl:loadComponent("traj_interp", "OCL::LuaComponent")
traj_interp = depl:getPeer("traj_interp")
traj_interp:exec_file(dir .. "/scripts/vel_traj_follow.lua")
-- depl:setMasterSlaveActivity("fbs", "traj_gen")
depl:setActivity("traj_interp", 0, 99, rtt.globals.ORO_SCHED_RT)
traj_interp:setPeriod(0.004)

depl:loadComponent("robot_sim", "OCL::LuaComponent")
robot_sim = depl:getPeer("robot_sim")
robot_sim:exec_file(dir .. "/scripts/simple_robot_sim.lua")
depl:setActivity("robot_sim", 0, 99, rtt.globals.ORO_SCHED_RT)
robot_sim:setPeriod(0.004)
j_init = rtt.Variable("array")
j_init:fromtab({-1.36542319, -0.74822507, 2.05658987, 0.52732208, 2.4950726,
 -0.93756902, -1.71694542, 1.32087, -0.77865726, -2.04601662, 0.65292945,
 -2.25832585, -0.81930464, 1.00047389})
robot_sim:getProperty("initial_position"):set(j_init)

-- Connecting the peers
depl:connectPeers("robot_sim","traj_interp")
depl:connectPeers("robot_sim","mpc")
depl:connectPeers("traj_interp","mpc")

-- Connecting the ports between the components
depl:connect("traj_interp.joint_vel_out_arr", "robot_sim.jointvel", cp)
depl:connect("robot_sim.jointpos", "mpc.q_actual", cp)
depl:connect("robot_sim.jointvel_out", "mpc.qdot_actual", cp)
depl:connect("traj_interp.joint_pos_in_ref", "mpc.q_command", cp)
depl:connect("traj_interp.joint_vel_in_ref", "mpc.qdot_command", cp)
depl:connect("traj_interp.joint_acc_in_ref", "mpc.qddot_command", cp)
ros:import("etasl_iohandler_jointstate")
depl:stream("robot_sim.jointpos", ros:topic("/joint_states_from_orocos"))
robot_sim:configure()
robot_sim:start()
--
--     --depl:connectPeers("yumi","fbs")
--     --depl:connect("yumi.triggerPort","fbs.trigger", cp)


-- --Reporting the joint velocity inputs and the recorded joint desired_positions
-- depl:loadComponent("Reporter", "OCL::FileReporting")
-- Reporter = depl:getPeer("Reporter")
-- depl:connectPeers("yumi", "Reporter");
-- Reporter:reportPort("yumi", "q_actual")
-- Reporter:reportPort("yumi", "q_dot_desired");
-- Reporter:getProperty("ReportFile"):set("right_1_8.dat")
-- --depl:setMasterSlaveActivity("fbs", "Reporter")
-- --sched_order=fbs:getProperty("sched_order")
-- --depl:connectPeers("Reporter","fbs")
-- ---depl:connectPeers("traj_gen","fbs")
-- --sched_order:get():resize(2)
-- --sched_order[0]="traj_gen"
-- --sched_order[1]="Reporter"
-- --fbs:start()
-- depl:setActivity("Reporter", 1/samplefreq, 99, rtt.globals.ORO_SCHED_RT)

--configure hook of both components
mpc:configure()
traj_interp:configure()
-- Reporter:configure()
--
-- yumi:start()
-- sleep(0.01) -- Some time lag because startHook in traj_gen might need to read some value from yumi component for starting.
-- traj_gen:start()
-- sleep(0.01) -- Some time lag because startHook in traj_gen might need to read some value from yumi component for starting.
-- Reporter:start()
--
--
--
--
-- print("configuration of the orocos components done")
--
--
-- --yumi:gripVacuumRight()
-- --sleep(3)
-- --yumi:gripVacuumRight()
--
traj_interp:start()
mpc:start()



sleep(5.0)
-- --
mpc:stop()
robot_sim:stop()
traj_interp:stop()
mpc:cleanup()

-- Reporter:stop()
print("Code finished successfully")
