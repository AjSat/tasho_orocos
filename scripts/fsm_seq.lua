require("rtt")
require("rttlib")

tc          = rtt.getTC()
depl        = tc:getPeer("Deployer")
move_joints = depl:getPeer("move_joints")
simrobot    = depl:getPeer("simrobot")
reporter    = depl:getPeer("Reporter")

pos1   = { 0/180*math.pi, -110/180*math.pi, 110/180*math.pi, -90/180*math.pi, -90/180*math.pi, 0/180*math.pi }
pos2   = { 0/180*math.pi, -120/180*math.pi, 135/180*math.pi, -115/180*math.pi, -90/180*math.pi, 0/180*math.pi }


return rfsm.state {
   configured = rfsm.state {
      entry=function()
            simrobot:configure()
            reporter:configure()
            simrobot:start()
            reporter:start()

            move_joints:configure()
          end,
   },

   moving_A = rfsm.state {
      entry=function()
            move_joints:set_etaslvar("global.maxvel",10.0/180.0*math.pi)
            move_joints:set_etaslvar("global.stop_j1",pos1[1])
            move_joints:set_etaslvar("global.stop_j2",pos1[2])
            move_joints:set_etaslvar("global.stop_j3",pos1[3])
            move_joints:set_etaslvar("global.stop_j4",pos1[4])
            move_joints:set_etaslvar("global.stop_j5",pos1[5])
            move_joints:set_etaslvar("global.stop_j6",pos1[6])
            move_joints:initialize()
            move_joints:start()
      end,
   },

   moving_B = rfsm.state {
      entry=function()
            move_joints:set_etaslvar("global.maxvel",10.0/180.0*math.pi)
            move_joints:set_etaslvar("global.stop_j1",pos2[1])
            move_joints:set_etaslvar("global.stop_j2",pos2[2])
            move_joints:set_etaslvar("global.stop_j3",pos2[3])
            move_joints:set_etaslvar("global.stop_j4",pos2[4])
            move_joints:set_etaslvar("global.stop_j5",pos2[5])
            move_joints:set_etaslvar("global.stop_j6",pos2[6])
            move_joints:initialize()
            move_joints:start()
      end,

   },

   finished = rfsm.state {
        entry = function()
        end,
   },

   rfsm.trans {src="initial", tgt="configured" },
   rfsm.trans {src="configured", tgt="moving_A", events={}},

   rfsm.trans {src="moving_A", tgt="finished", events={"e_stop"}},
   rfsm.trans {src="moving_B", tgt="finished", events={"e_stop"}},

   rfsm.trans {src="moving_A", tgt="moving_B", events={"e_finished@move_joints"}},
   rfsm.trans {src="moving_B", tgt="moving_A", events={"e_finished@move_joints"}},
}
