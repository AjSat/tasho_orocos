#include "OCPComponent.hpp"

#define M_PI 3.14159265358979323846
#define TIMEOUT 500 // [ms]


    OCPComponent::OCPComponent(const string &name) : TaskContext(name, PreOperational), p_numjoints(14), horizon(15), degrees_to_radians(M_PI / 180.0),  ocp_rate(10), time(0.0), wait(true)
    {
      //Adding properties
      this->addProperty("ocp_rate", ocp_rate).doc("Sampling rate of the OCP");
      this->addProperty("horizon", horizon).doc("Horizon size of the MPC");
      this->addProperty("ocp_file", ocp_file).doc("The casadi file that will compute the OCP.");
      this->addProperty("qdes", p_qdes).doc("desired final position of the robot.");
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
        f_ret = casadi_c_push_file(ocp_file.c_str());
        if (f_ret) {
          cout << "Failed to load the ocp file " + ocp_file;
          return -1;
        }
        // Identify a Function by name
        f_id = casadi_c_id("ocp_fun");
        n_in = casadi_c_n_in_id(f_id);
        n_out = casadi_c_n_out_id(f_id);

        sz_arg=n_in; sz_res=n_out; sz_iw=0; sz_w=0;

        casadi_c_work_id(f_id, &sz_arg, &sz_res, &sz_iw, &sz_w);
        printf("Work vector sizes:\n");
        printf("sz_arg = %lld, sz_res = %lld, sz_iw = %lld, sz_w = %lld\n\n",
            sz_arg, sz_res, sz_iw, sz_w);
        Logger::log() << Logger::Debug << "Got work ids" << Logger::endl;

            /* Allocate input/output buffers and work vectors*/

        res = new double*[sz_res];
        iw = new casadi_int[sz_iw];
        w = new double[70000];

        /* Function input and output */
        //parameters that need to be set a0, q_dot0, s0, s_dot0 TODO: read all from orocos ports
        double *q0 = new double[14];
        double *q_dot0 = new double[14];

        // Assign a fixed size and memory to the vectors associated with the ports
        m_q_actual.assign(p_numjoints, 0);
        m_qdot_actual.assign(p_numjoints, 0);
        m_q_command.assign(p_numjoints, 0);
        m_qd_command.assign(p_numjoints, 0);
        m_qdd_command.assign(p_numjoints, 0);

        if (port_q_actual.read(m_q_actual) != NoData){
          // Logger::log() << Logger::Debug << "Read joint pos from robot_sim" << Logger::endl;
          for(int i = 0; i<14; i++){
            // Logger::log() << Logger::Debug << "Initializing the joint values = " << m_q_actual[i] << Logger::endl;
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
          Logger::log() << Logger::Error << "Failed to read robot velocities" << Logger::endl;
          return false;
        }

        x_val = new double[3000];
        x_val2 = new double[3000];
        res0 = new double[3000];
        res2 = new double[3000];
        for(int i = 0; i < 3000; i++){
          x_val[i] = 0;
          x_val2[i] = 0;
          res0[i] = 0;
          res2[i] = 0;
        }

        //Initilializing the parameters to the correct values of x
        int q0_start = 654;
        int q_start = 0;
        int q_size = 14;
        for(int i = 0; i < q_size; i++){
          x_val[q0_start + i] = q0[i];
          for(int j = 0; j < horizon + 1; j++){
            x_val[q_start + j*(horizon + 1) + i] = q0[i];
          }
        }

        //TODO: add the correct pdes to the parameters

        // Allocate memory (thread-safe)
        Logger::log() << Logger::Debug << "Allocating memory" << Logger::endl;
        casadi_c_incref_id(f_id);

        /* Evaluate the function */
        arg[0] = x_val;
        arg[1] = x_val2;
        res[0] = res0;
        res[1] = res2;
        Logger::log() << Logger::Debug << "Creating arguments for casadi function" << Logger::endl;

        // Checkout thread-local memory (not thread-safe)
        mem = casadi_c_checkout_id(f_id);

        // Evaluation is thread-safe TODO: add error handling if the OCP fails to find a solution
        Logger::log() << Logger::Debug << "Evaluating casadi OCP function" << Logger::endl;

        casadi_c_eval_id(f_id, arg, res, iw, w, mem);

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

      Logger::log() << Logger::Debug << "Entering updateHook" << Logger::endl;
      //Apply the control inputs
      //read values for q_command, q_d_command and q_dd_command
      for(int i = 0; i < 14; i++){
        m_q_command[i] = res0[i + sequence*14];
        m_qd_command[i] = res0[196 + i + sequence*14];
        m_qdd_command[i] = res0[420 + i + sequence*14];
      }
      //Write the q, qd and qdd commands into the respective ports
      port_q_command.write(m_q_command);
      port_qdot_command.write(m_qd_command);
      port_qddot_command.write(m_qdd_command);
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
