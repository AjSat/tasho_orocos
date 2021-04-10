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
dir = rtt.provides("ros"):find("yumi_tasho")
mpc:getProperty("ocp_file"):set(dir .. "/casadi_files/ocp_fun.casadi")
mpc:getProperty("mpc_file"):set(dir .. "/casadi_files/mpc_fun.casadi")
mpc:getProperty("predict_file"):set(dir .. "/casadi_files/jac_fun_rob.casadi")
mpc:getProperty("shift_file"):set(dir .. "/casadi_files/jac_fun_rob.casadi")

depl:setActivity("mpc", 0, 99, rtt.globals.ORO_SCHED_RT)
mpc:setPeriod(0.05)
cp = rtt.Variable("ConnPolicy")

-- depl:loadComponent("traj_gen", "OCL::LuaComponent")
-- traj_gen = depl:getPeer("traj_gen")
-- --depl:setMasterSlaveActivity("fbs", "traj_gen")
--
-- if control_mode == 0 then
--
--     traj_gen:exec_file(dir .. "joint_velocity_manual.lua")
--     traj_gen:setPeriod(0.004)
--
--     --depl:connectPeers("yumi","fbs")
--     --depl:connect("yumi.triggerPort","fbs.trigger", cp)
--     --Connecting the ports
--
--     depl:connect("yumi.JointVelocityCommand", "traj_gen.desired_velocities", cp)
--     depl:connect("yumi.q_actual", "traj_gen.measured_angles", cp)
--
--
-- elseif control_mode == 1 then
--
--     traj_gen:exec_file(dir .. "joint_position_manual.lua")
--     traj_gen:setPeriod(0.004)
--
--     --connecting the ports
--
--     depl:connect("yumi.JointPositionCommand", "traj_gen.desired_positions", cp)
--     depl:connect("yumi.q_actual", "traj_gen.measured_angles", cp)
--
-- end
--
-- --Reporting the joint velocity inputs and the recorded joint desired_positions
--
--
--
--
-- depl:loadComponent("Reporter", "OCL::FileReporting")
-- Reporter = depl:getPeer("Reporter")
--
-- depl:connectPeers("yumi", "Reporter");
-- Reporter:reportPort("yumi", "q_actual")
-- Reporter:reportPort("yumi", "q_dot_desired");
-- Reporter:getProperty("ReportFile"):set("right_1_8.dat")
--
-- --depl:setMasterSlaveActivity("fbs", "Reporter")
--
-- --sched_order=fbs:getProperty("sched_order")
-- --depl:connectPeers("Reporter","fbs")
-- ---depl:connectPeers("traj_gen","fbs")
-- --sched_order:get():resize(2)
-- --sched_order[0]="traj_gen"
-- --sched_order[1]="Reporter"
-- --fbs:start()
--
-- depl:setActivity("Reporter", 1/samplefreq, 99, rtt.globals.ORO_SCHED_RT)
--
-- print("created reported and configured ports for reporting")
--
-- print("connected ports of traj_gen component to orocos driver ports");
--
--configure hook of both components
mpc:configure()
-- traj_gen:configure()
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
mpc:start()
sleep(2)
--
mpc:stop()
-- traj_gen:stop()
-- Reporter:stop()
--
--
--
-- print("here")
