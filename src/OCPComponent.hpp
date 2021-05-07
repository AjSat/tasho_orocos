
#ifndef __OCP_COMPONENT_H__
#define __OCP_COMPONENT_H__

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

  class OCPComponent : public TaskContext
  {
  public:
    OCPComponent(const std::string &name);
    virtual ~OCPComponent();

    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

  //private:

    //Functions
    void port_writer();
    /// Properties
    unsigned int p_numjoints;
    double p_max_vel, p_max_acc;
    string ocp_file, ocp_fun;
    const double degrees_to_radians;
    double Kpos;
    // TODO: egm_rate is currently not used!
    double ocp_rate;
    int horizon, q0_start, qdot0_start, q_start, q_dot_start, q_ddot_start, goal_start, max_vel_loc, max_acc_loc;

    //const unsigned int egm_rate;  // [Hz] (EGM communication rate, specified by the EGMActJoint RAPID instruction)
    int sequence = 0; // [-] (sequence number of a received EGM message)
    double time;                  // [seconds] (elapsed time during an EGM communication session)

    int i, f_id, f_ret, mem;
    casadi_int n_in, n_out, sz_arg, sz_res, sz_iw, sz_w, *iw;
    double  *w, *x_val, *x_val2, *res0, *res2;
    double **res;
    const double **arg;
    bool wait, flag = true;
    bool first_message, p_left_arm, p_joint_space;



    // Internal, mem alloc
    sensor_msgs::JointState m_joint_states;
    vector<double> p_qdes;
    vector<double> p_fk_des;
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

#endif // __OCP_COMPONENT_H__
