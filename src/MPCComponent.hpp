
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
    bool p_simulation;
    const unsigned int p_numjoints;
    string p_egm_ip;
    unsigned int p_egm_port;
    string p_baseframe;
    string p_effort_origin;
    string ocp_file;
    string mpc_file;
    string predict_file;
    string shift_file;
    const double degrees_to_radians;
    double Kpos;
    // TODO: egm_rate is currently not used!
    double mpc_rate;
    int horizon;

    //const unsigned int egm_rate;  // [Hz] (EGM communication rate, specified by the EGMActJoint RAPID instruction)
    unsigned int sequence_number; // [-] (sequence number of a received EGM message)
    double time;                  // [seconds] (elapsed time during an EGM communication session)

    int f_id, f_ret, mem;
    casadi_int n_in, n_out, sz_arg, sz_res, sz_iw, sz_w, *iw;
    double  *w, *x_val, *x_val2, *res0, *res2;
    double **res;
    const double *arg[250]; //TODO: hardcoded, hope that it is always high enough
    bool wait, flag = true;
    bool first_message;



    // Internal, mem alloc
    geometry_msgs::Pose m_cart_pose;
    sensor_msgs::JointState m_joint_states;
    vector<double> m_qdes;
    vector<double> m_q_actual;
    vector<double> m_t_actual;
    vector<double> m_qdot_actual;
    vector<double> m_q_command;
    vector<double> m_qd_command;
    vector<double> m_qdd_command;

    // Port Interface
    InputPort<string> port_ein;
    InputPort<vector<double>> port_qdes;
    InputPort<vector<double>> port_qdot_actual;
    InputPort<vector<double>> port_q_actual;

    // Output port
    OutputPort<string> port_eout;
    OutputPort<vector<double>> port_qdot_command;
    OutputPort<vector<double>> port_q_command;
    OutputPort<vector<double>> port_qddot_command;
    OutputPort<vector<double>> port_t_actual;
    // OutputPort<motion_control_msgs::JointEfforts> port_joint_ext_jnt;
    // OutputPort<sensor_msgs::JointState> port_joint_state;
    // OutputPort<geometry_msgs::Pose> port_cart_pose;
  };

#endif // __MPC_COMPONENT_H__
