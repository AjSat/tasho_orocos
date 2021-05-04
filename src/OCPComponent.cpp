#include "OCPComponent.hpp"

#define M_PI 3.14159265358979323846
#define TIMEOUT 500 // [ms]


    OCPComponent::OCPComponent(const string &name) : TaskContext(name, PreOperational), p_numjoints(14), horizon(15), degrees_to_radians(M_PI / 180.0),  ocp_rate(10), time(0.0), wait(true), p_max_vel(20.0/180*3.14159), p_joint_space(true), p_max_acc(120/180*3.14159), p_left_arm(true)
    {
      //Adding properties
      this->addProperty("ocp_rate", ocp_rate).doc("Sampling rate of the OCP");
      this->addProperty("num_joints", p_numjoints).doc("Number of joints");
      this->addProperty("horizon", horizon).doc("Horizon size of the MPC");
      this->addProperty("ocp_file", ocp_file).doc("The casadi file that will compute the OCP.");
      this->addProperty("qdes", p_qdes).doc("desired final position of the robot.");
      this->addProperty("fk_des", p_fk_des).doc("Desired final pose of the robot arm. (12,1)");
      this->addProperty("move_left_arm", p_left_arm).doc("Set to true to move left arm,false for right arm.");
      this->addProperty("joint_pos", p_joint_space).doc("Set to true if goal is in jointspace. False if Cartesian.");
      this->addProperty("max_vel", p_max_vel).doc("Maximum limits on joint velocities (rad/s)");
      this->addProperty("max_acc", p_max_acc).doc("Maximum limit on acceleration (rad/s^2)");
      //Adding ports
      /// Input
      this->addPort("event_in", port_ein).doc("Events IN - eg supervisor");
      this->addPort("q_actual", port_q_actual).doc("current joint positions [rad]");
      this->addPort("qdot_actual", port_qdot_actual).doc("current joint velocities [rad/s]");
      /// Output
      this->addPort("event_out", port_eout).doc("Events OUT - eg faults to supervisor");
      this->addPort("q_command", port_q_command).doc("Desired joint positions [rad]");
      this->addPort("qdot_command", port_qdot_command).doc("Desired joint velocities [rad/s]");
      this->addPort("qddot_command", port_qddot_command).doc("Desired joint accelerations [rad/s^2]");


      //Sanity check on integer types TODO: throw error if fails and move to header
      if (casadi_c_int_width()!=sizeof(casadi_int)) {
        printf("Mismatch in integer size\n");
      }
      if (casadi_c_real_width()!=sizeof(double)) {
        printf("Mismatch in double size\n");
      }

    }

    OCPComponent::~OCPComponent()
    {
    }

    bool OCPComponent::configureHook()
    {

        Logger::In in(this->getName());
        Logger::log() << Logger::Debug << "Entering configuration hook" << Logger::endl;

        ocp_function = casadi::Function::load(ocp_file);

        f_ret = casadi_c_push_file(ocp_file.c_str());
        if (f_ret) {
          cout << "Failed to load the ocp file " + ocp_file;
          return -1;
        }
        // Identify a Function by name
        // f_id = casadi_c_id("ocp_fun");
        // n_in = casadi_c_n_in_id(f_id);
        // n_out = casadi_c_n_out_id(f_id);

        // sz_arg=n_in; sz_res=n_out; sz_iw=0; sz_w=0;

        n_in = ocp_function.n_in();
        n_out = ocp_function.n_out();
        sz_arg = ocp_function.sz_arg();
        sz_res = ocp_function.sz_res();
        sz_iw = ocp_function.sz_iw();
        sz_w = ocp_function.sz_w();

        
        // casadi_c_work_id(f_id, &sz_arg, &sz_res, &sz_iw, &sz_w);
        printf("Work vector sizes:\n");
        printf("sz_arg = %lld, sz_res = %lld, sz_iw = %lld, sz_w = %lld\n\n",
            sz_arg, sz_res, sz_iw, sz_w);
        Logger::log() << Logger::Debug << "Got work ids" << Logger::endl;

            /* Allocate input/output buffers and work vectors*/

        // res = new double*[sz_res];
        // iw = new casadi_int[sz_iw];
        // w = new double[sz_w];

        arg.resize(sz_arg);
        res.resize(sz_res);
        iw.resize(sz_iw);
        w.resize(sz_w);

        /* Function input and output */
        //parameters that need to be set a0, q_dot0, s0, s_dot0 TODO: read all from orocos ports
        double *q0 = new double[p_numjoints];
        double *q_dot0 = new double[p_numjoints];

        // Assign a fixed size and memory to the vectors associated with the ports
        m_q_actual.assign(14, 0);
        m_qdot_actual.assign(14, 0);
        m_q_command.assign(14, 0);
        m_qd_command.assign(14, 0);
        m_qdd_command.assign(14, 0);

        if (port_q_actual.read(m_q_actual) != NoData){
          Logger::log() << Logger::Debug << "Read joint pos from robot_sim" << Logger::endl;
          for(int i = 0; i<14; i++){
            Logger::log() << Logger::Debug << "Initializing the joint values = " << m_q_actual[i] << Logger::endl;
            q0[i] = m_q_actual[i];
          }
        }
        else{
          Logger::log() << Logger::Error << "Failed to read robot joint positions" << Logger::endl;
          return false;
        }
        if (port_qdot_actual.read(m_qdot_actual) != NoData){
          // Logger::log() << Logger::Debug << "Read joint vel from robot_sim" << Logger::endl;
          for(int i = 0; i<14; i++){
            // Logger::log() << Logger::Debug << "Initializing the joint vel values = " << m_qdot_actual[i] << Logger::endl;
            q_dot0[i] = m_qdot_actual[i];
          }
        }
        else{
          Logger::log() << Logger::Debug
          << "Failed to read robot velocities. Assigning zero values." << Logger::endl;
          double q_dot0_def[p_numjoints] = {0,0,0,0,0,0,0,
            0,0,0,0,0,0,0};
          q_dot0 = q_dot0_def;
        }

        x_val = new double[5000];
        x_val2 = new double[5000];
        res0 = new double[5000];
        res2 = new double[5000];
        for(int i = 0; i < 5000; i++){
          x_val[i] = 0;
          x_val2[i] = 0;
          res0[i] = 0;
          res2[i] = 0;
        }
        Logger::log() << Logger::Debug << "declared variables" << Logger::endl;

        if(p_joint_space){
          q0_start = 1710;
          q_start = 0;
          q_dot_start = 574;
          q_ddot_start = 1148;
          max_vel_loc = 1708;
          max_acc_loc = 1709;
          goal_start = 1724;
        }
        else{
          q_start = 0;
          q0_start = 856;
          q_dot_start = 287;
          q_ddot_start = 574;
          max_vel_loc = 854;
          max_acc_loc = 855;
          goal_start = 863;
        }

        //Initilializing the parameters to the correct values of x
        for(int i = 0; i < p_numjoints; i++){
          x_val[q0_start + i] = q0[i];
          for(int j = 0; j < horizon + 1; j++){
            x_val[q_start + j*(horizon + 1) + i] = q0[i];
          }
        }

        Logger::log() << Logger::Debug << "Initialized decision" << Logger::endl;

        //Adding the desired final joint position
        if(p_joint_space){
          for(int i = 0; i < p_numjoints; i++){
            x_val[goal_start + i] = p_qdes[i];
          }
        }
        else{
          for(int i = 0; i < 12; i++){
            x_val[goal_start + i] = p_fk_des[i];
          }
        }

        x_val[max_acc_loc] = p_max_acc; //setting maximum acceleration.
        x_val[max_vel_loc] = p_max_vel; //setting maximum velocity

        // Allocate memory (thread-safe)
        Logger::log() << Logger::Debug << "Allocating memory" << Logger::endl;
        casadi_c_incref_id(f_id);

        /* Evaluate the function */
        arg[0] = x_val;
        arg[1] = x_val2;
        res[0] = res0;
        res[1] = res2;
        // arg[0] = casadi::get_ptr(x_val);
        // arg[1] = casadi::get_ptr(x_val2);
        // res[0] = casadi::get_ptr(res0);
        // res[1] = casadi::get_ptr(res2);
        Logger::log() << Logger::Debug << "Creating arguments for casadi function" << Logger::endl;

        // Checkout thread-local memory (not thread-safe)
        // mem = casadi_c_checkout_id(f_id);
        mem = ocp_function.checkout();

        // Evaluation is thread-safe TODO: add error handling if the OCP fails to find a solution
        Logger::log() << Logger::Debug << "Evaluating casadi OCP function" << Logger::endl;

        // for(int i = 0; i < 14; i++){
        //   m_qdd_command[i] = 0.0;
        // }

        // bool solver_status = casadi_c_eval_id(f_id, arg, res, iw, w, mem);
        int solver_status = ocp_function(arg, res, iw, w, mem);
        // int solver_status = ocp_function(casadi::get_ptr(arg),casadi::get_ptr(res),casadi::get_ptr(iw),casadi::get_ptr(w), mem);

        Logger::log() << Logger::Error << "Solver status = " << solver_status << Logger::endl;
        if(solver_status){
          Logger::log() << Logger::Error << "OCP computation failed" << Logger::endl;
          return false;
        }

        Logger::log() << Logger::Debug << "Exiting configuration hook" << Logger::endl;
        return true;
    }

    bool OCPComponent::startHook()
    {

            Logger::log() << Logger::Debug << "Entering startHook" << Logger::endl;
            Logger::In in(this->getName());

        return true;
    }

    void OCPComponent::updateHook()
    {

      Logger::log() << Logger::Debug << "Entering UpdateHook" << Logger::endl;
      //Apply the control inputs
      //read values for q_command, q_d_command and q_dd_command
      // Logger::log() << Logger::Debug << (!p_joint_space && !p_left_arm) << Logger::endl;
      if(sequence < horizon){
        if(p_joint_space){
          for(i = 0; i < p_numjoints; i++){
            m_q_command[i] = res0[i +  sequence*p_numjoints];
            m_qd_command[i] = res0[q_dot_start + i + sequence*p_numjoints];
            m_qdd_command[i] = res0[q_ddot_start + i + sequence*p_numjoints];
          }
        }
        else{
          if (p_left_arm){
            for(i = 0; i < 7; i++){
              m_q_command[i] = res0[i + sequence*7];
              m_qd_command[i] = res0[q_dot_start +  i + sequence*7];
              m_qdd_command[i] = res0[q_ddot_start + i + sequence*7];
            }
          }
          else{
            // Logger::log() << Logger::Debug << "Updating right arm velocities for sequence " << sequence << Logger::endl;
            for(i = 0; i < 7; i++){
              m_q_command[i + 7] = res0[i + sequence*7];
              m_qd_command[i + 7] = res0[q_dot_start  +  i + sequence*7];
              m_qdd_command[i+7] = res0[q_ddot_start + i + sequence*7];
            }
            // Logger::log() << Logger::Debug << m_qd_command[7] << Logger::endl;
          }
        }
      }
      else if(sequence == horizon){
        for(i = 0; i < 14; i++){
          m_qd_command[i] = 0;
          m_qdd_command[i] = 0;
        }
        Logger::log() << Logger::Debug << "OCP finished: writing event" << Logger::endl;
        port_eout.write(ocp_file + "_ocp_done");
      }
      //Write the q, qd and qdd commands into the respective ports
      port_q_command.write(m_q_command);
      // Logger::log() << Logger::Debug << "Writing into velocity port for sequence " << sequence <<  Logger::endl;
      port_qdot_command.write(m_qd_command);
      port_qddot_command.write(m_qdd_command);
      sequence++;
      this->trigger(); //TODO: shouldn't this trigger be removed?
    }

    void OCPComponent::stopHook()
    {
      Logger::log() << Logger::Debug << "Entering the stopHook" << Logger::endl;
    }

    void OCPComponent::cleanupHook()
    {
      Logger::log() << Logger::Debug << "Entering the cleanupHook" << Logger::endl;
      casadi_c_decref_id(f_id);
      // casadi_c_pop();
      Logger::log() << Logger::Debug << "Exiting the cleanupHook" << Logger::endl;

    }

ORO_CREATE_COMPONENT(OCPComponent)
