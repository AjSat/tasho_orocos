-- Shows the rFSM used in the sequence of pick and place tasks in the previous directory

require("rtt")
require("rttlib")

tc          = rtt.getTC()
depl        = tc:getPeer("Deployer")
sup = depl:getPeer("Supervisor")
ocp = depl:getPeer("ocp")
ocp_homing = depl:getPeer("ocp_homing")
traj_interp = depl:getPeer("traj_interp")
robot_sim = depl:getPeer("robot_sim")

function sleep(n)
  os.execute("sleep " .. tonumber(n))
end

return rfsm.state {
   configured = rfsm.state {
      entry=function()
            print("This ran")
            robot_sim:configure()
            robot_sim:start()
            traj_interp:configure()
            traj_interp:start()
          end,
   },

   moving_A = rfsm.state {
      entry=function()
        fk_des = rtt.Variable("array")
        fk_des:fromtab({ 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.4, 0.4, 0.4}) --left
        ocp:getProperty("fk_des"):set(fk_des)
        ocp:configure()
        ocp:start()
      end,
   },

   finishedA = rfsm.state {
     entry=function()
       ocp:stop()
     end,
   },

   moving_B = rfsm.state {
      entry=function()
        home_pos = rtt.Variable("array")
        print("This ran")
        home_pos:fromtab({ -0.19690, -2.33, 1.95, 0.6580, 0.2390, 0.3770, -0.4250, 0, -2.26, -2.35, 0.52, 0.025, 0.749, 0,})
        print("This ran2")
        ocp_homing:getProperty("qdes"):set(home_pos)
        print("This ran3")
        ocp_homing:configure()
        print("This ran4")
        ocp_homing:start()
      end,

   },

   finishedB = rfsm.state {
        entry = function()
          ocp_homing:stop()
          robot_sim:stop()
          traj_interp:stop()
        end,
   },

   rfsm.trans {src="initial", tgt="configured" },
   rfsm.trans {src="configured", tgt="moving_A", events={}},

   rfsm.trans {src="moving_A", tgt="finishedA", events={"leftp2p_ocp_fun_done"}},
   rfsm.trans {src="finishedA", tgt="moving_B", events={}},
   rfsm.trans {src="moving_B", tgt="finishedB", events={"homing_ocp_fun_done"}},

}
