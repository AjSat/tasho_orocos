
#ifndef __MPC_COMPONENT_H__
#define __MPC_COMPONENT_H__

//casadi specific
#include <stdio.h>
#include <casadi/casadi_c.h>

// OROCOS-rtt specific
#include <rtt/RTT.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/os/Mutex.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/MutexLock.hpp>

// ros_rttcom
#include <sensor_msgs/JointState.h>
#include <motion_control_msgs/JointAccelerations.h>
#include <motion_control_msgs/JointPositions.h>
#include <motion_control_msgs/JointEfforts.h>
#include <motion_control_msgs/JointVelocities.h>
#include <geometry_msgs/Pose.h>

#include <vector>
// Including the json cpp parser
#include "json.hpp"
using json = nlohmann::json;

  using namespace RTT;
  using namespace std;

  class MPCComponent : public TaskContext
  {
  public:
    MPCComponent(const std::string &name);
    virtual ~MPCComponent();

    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

  private:

    //Functions
    void port_writer();
    /// Properties
    int num_inp_ports, num_out_ports, num_states, num_controls;
    string p_js_prop_file;
    string p_ocp_file;
    string p_mpc_file;
    string p_predict_file;
    const double degrees_to_radians;
    int p_horizon, p_term_cond_pos;
    casadi_int nnz, nnz_out;

    unsigned int sequence_number; // [-] (sequence number of a received EGM message)
    double time;                  // [seconds] (elapsed time during an EGM communication session)

    int f_id, f_ret, mem, memp, pred_f_id;
    casadi_int n_in, n_out, sz_arg, sz_res, sz_iw, sz_w, *iw, *iwp;
    double  *w, *wp, *x_val, *res0;
    double **res, **resp;
    const double **arg, **argp; //TODO: hardcoded, hope that it is always high enough
    bool wait, flag = true;
    bool first_message, terminated;

    json jsp; //json object to parse the MPC component properties

    // Internal, mem alloc
    vector<double> *m_inp_ports;
    vector<double> *m_out_ports;
    vector<double> *vector_props;

    // Port Interface
    InputPort<string> port_ein;
    InputPort<vector<double>> *inp_ports;
    std::string out_port_var;

    // Output port
    OutputPort<string> port_eout;
    OutputPort<vector<double>> *out_ports;
  };

#endif // __MPC_COMPONENT_H__
