-- A lua file that shows using Tasho to deploy a sequence of actions and monitors
-- using FSM.

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
depl:import("rtt_sensor_msgs")
depl:import("rtt_motion_control_msgs")

depl:loadComponent("ocp", "OCPComponent")
ocp = depl:getPeer("ocp")
ros:import("etasl_iohandler_jointstate")
--Configuration
--6511 is ROB_L 6512 is ROB_R
ocp:getProperty("ocp_rate"):set(10) -- in Hz
ocp:getProperty("num_joints"):set(7) -- in Hz
ocp:getProperty("horizon"):set(40) -- number of sampling steps
dir = ros:find("tasho_orocos")
ocp:getProperty("ocp_file"):set(dir .. "/casadi_files/leftp2p_ocp_fun.casadi")
ocp:getProperty("ocp_fun"):set("leftp2p_ocp_fun")

depl:loadComponent("ocp_homing", "OCPComponent")
ocp_homing = depl:getPeer("ocp_homing")
ocp_homing:getProperty("ocp_rate"):set(10) -- in Hz
ocp_homing:getProperty("horizon"):set(40) -- in Hz
ocp_homing:getProperty("ocp_file"):set(dir .. "/casadi_files/homing_ocp_fun.casadi")
ocp_homing:getProperty("ocp_fun"):set("homing_ocp_fun")

-- fk_des = rtt.Variable("array")
-- -- fk_des:fromtab({ 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.4, 0.4, 0.4}) --left
-- fk_des:fromtab({ 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.4, 0.4, 0.4}) --left
-- ocp:getProperty("fk_des"):set(fk_des)
ocp:getProperty("max_vel"):set(120/180.0*3.14159)
ocp:getProperty("max_acc"):set(240/180*3.14159)
ocp:getProperty("joint_pos"):set(false)
ocp:getProperty("move_left_arm"):set(true)
depl:setActivity("ocp", 0, 99, rtt.globals.ORO_SCHED_RT)
ocp:setPeriod(0.1)

-- home_pos = rtt.Variable("array")
-- home_pos:fromtab({ -0.19690, -2.33, 1.95, 0.6580, 0.2390, 0.3770, -0.4250, 0, -2.26, -2.35, 0.52, 0.025, 0.749, 0,})
-- ocp_homing:getProperty("qdes"):set(home_pos)
ocp_homing:getProperty("max_vel"):set(50/180.0*3.14159)
ocp_homing:getProperty("max_acc"):set(120/180*3.14159)
ocp_homing:getProperty("joint_pos"):set(true)
depl:setActivity("ocp_homing", 0, 99, rtt.globals.ORO_SCHED_RT)
ocp_homing:setPeriod(0.1)

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
j_init:fromtab({-1.36542319, -0.74822507, 2.05658987, 0.52732208, 2.4950726,
 -0.93756902, -1.71694542, 1.32087, -0.77865726, -2.04601662, 0.65292945,
 -2.25832585, -0.81930464, 1.00047389})
robot_sim:getProperty("initial_position"):set(j_init)

-- deploy supervisor:
  depl:loadComponent("Supervisor", "OCL::LuaComponent")
  sup = depl:getPeer("Supervisor")
  sup:exec_file(dir.."/scripts/rfsm/fsm_component.lua")
  sup:getProperty("state_machine"):set(dir.."/scripts/rfsm/fsm_seq.lua")
  sup:addPeer(depl)
  sup:configure()
  sup:start()
  cmd = rttlib.port_clone_conn(sup:getPort("events"))

-- Connecting the peers
depl:connectPeers("robot_sim","traj_interp")
depl:connectPeers("robot_sim","ocp")
depl:connectPeers("traj_interp","ocp")
depl:connectPeers("robot_sim","ocp_homing")
depl:connectPeers("traj_interp","ocp_homing")



-- Connecting the ports between the components
depl:connect("traj_interp.joint_vel_out_arr", "robot_sim.jointvel", cp)
depl:connect("traj_interp.joint_pos_in_actual", "robot_sim.jointpos", cp)
depl:connect("robot_sim.jointpos", "ocp.q_actual", cp)
depl:connect("robot_sim.jointvel_out", "ocp.qdot_actual", cp)
depl:connect("traj_interp.joint_pos_in_ref", "ocp.q_command", cp)
depl:connect("traj_interp.joint_vel_in_ref", "ocp.qdot_command", cp)
depl:connect("traj_interp.joint_acc_in_ref", "ocp.qddot_command", cp)
depl:stream("robot_sim.jointpos", ros:topic("/joint_states_from_orocos"))

depl:connect("robot_sim.jointpos", "ocp_homing.q_actual", cp)
depl:connect("robot_sim.jointvel_out", "ocp_homing.qdot_actual", cp)
depl:connect("traj_interp.joint_pos_in_ref", "ocp_homing.q_command", cp)
depl:connect("traj_interp.joint_vel_in_ref", "ocp_homing.qdot_command", cp)
depl:connect("traj_interp.joint_acc_in_ref", "ocp_homing.qddot_command", cp)

depl:connect("ocp.event_out","Supervisor.events",cp)
depl:connect("ocp_homing.event_out","Supervisor.events",cp)
-- robot_sim:configure()
-- robot_sim:start()
-- traj_interp:configure()
-- traj_interp:start()
--
-- --configure hook of both components
--
-- ocp:configure()
-- ocp:start()
-- sleep(10.0)
-- ocp:stop()
--
--
--
-- ocp_homing:configure()
-- ocp_homing:start()
-- sleep(6.0)
-- ocp_homing:stop()
--
-- robot_sim:stop()
-- traj_interp:stop()
-- ocp:cleanup()
-- ocp_homing:cleanup()

-- Reporter:stop()
sleep(20);
sup:stop()
print("Code finished successfully")
