#include "OCPComponent.hpp"

#define M_PI 3.14159265358979323846
#define TIMEOUT 500 // [ms]


    OCPComponent::OCPComponent(const string &name) : TaskContext(name), p_horizon(15), time(0.0), wait(true)
    {

      this->addProperty("js_prop_file", p_js_prop_file).doc("The location to the json file containing all the info pertaining to the OCP.");
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

    bool OCPComponent::configureHook(){
      //Adding properties
      Logger::In in(this->getName());
      Logger::log() << Logger::Info << "Entering configuration hook" << Logger::endl;
      FILE * pFile;
      pFile = fopen (p_js_prop_file.c_str(), "r");
      if (pFile == NULL) printf("Error opening file");
      js_prop = json::parse(pFile);

      double_props.resize(3, 0.0);
      integer_props.resize(2, 0); // = new int[2];
      vector_props = new vector<double>[2];
      this->addProperty("ocp_rate", double_props[2]).doc("Sampling rate of the OCP");
      this->addProperty("num_joints", integer_props[0]).doc("Number of joints");
      this->addProperty("goal_des", vector_props[0]).doc("Desired final pose of the robot arm.");
      this->addProperty("joint_pos", integer_props[1]).doc("Set to true if goal is in jointspace. False if Cartesian.");
      this->addProperty("max_vel", double_props[0]).doc("Maximum limits on joint velocities (rad/s)");
      this->addProperty("max_acc", double_props[1]).doc("Maximum limit on acceleration (rad/s^2)");
      // p_numjoints = 14;
      //Adding ports
      /// Input
      this->addPort("event_in", port_ein).doc("Events IN - eg supervisor");
      // inp_ports.reserve(2);
      inp_ports = new InputPort<vector<double>>[2];
      this->addPort("q_actual", inp_ports[0]).doc("current joint positions [rad]");
      this->addPort("qdot_actual", inp_ports[1]).doc("current joint velocities [rad/s]");
      /// Output

      this->addPort("event_out", port_eout).doc("Events OUT - eg faults to supervisor");
      out_ports = new OutputPort<vector<double>>[3];
      this->addPort("q_command", out_ports[0]).doc("Desired joint positions [rad]");
      this->addPort("qdot_command", out_ports[1]).doc("Desired joint velocities [rad/s]");
      this->addPort("qddot_command", out_ports[2]).doc("Desired joint accelerations [rad/s^2]");

      p_horizon = js_prop["horizon"].get<int>();
      p_ocp_file = js_prop["casadi_fun"].get<std::string>();
      p_ocp_fun = js_prop["fun_name"].get<std::string>();

      f_ret = casadi_c_push_file(p_ocp_file.c_str());
      Logger::log() << Logger::Info << "Loaded configuration file" << Logger::endl;
      if (f_ret) {
        cout << "Failed to load the ocp file " + p_ocp_file;
        return -1;
      }
      // Identify a Function by name
      f_id = casadi_c_id(p_ocp_fun.c_str());
      n_in = casadi_c_n_in_id(f_id);
      n_out = casadi_c_n_out_id(f_id);

      sz_arg=n_in; sz_res=n_out; sz_iw=0; sz_w=0;

      const casadi_int *sp_i;
      sp_i = casadi_c_sparsity_in_id(f_id, 0);
      casadi_int nrow = *sp_i++; /* Number of rows */
      casadi_int ncol = *sp_i++; /* Number of columns */
      casadi_int nnz = sp_i[ncol]; /* Number of nonzeros */

      sp_i = casadi_c_sparsity_out_id(f_id, 0);
      nrow = *sp_i++; /* Number of rows */
      ncol = *sp_i++; /* Number of columns */
      casadi_int nnz_out = sp_i[ncol]; /* Number of nonzeros */

      sz_arg=n_in; sz_res=n_out; sz_iw=0; sz_w=0;

      casadi_c_work_id(f_id, &sz_arg, &sz_res, &sz_iw, &sz_w);
      printf("Work vector sizes:\n");
      printf("sz_arg = %lld, sz_res = %lld, sz_iw = %lld, sz_w = %lld\n\n",
          sz_arg, sz_res, sz_iw, sz_w);
      Logger::log() << Logger::Debug << "Got work ids" << Logger::endl;

          /* Allocate input/output buffers and work vectors*/

      res = new double*[sz_res];
      arg = new const double*[sz_arg];
      iw = new casadi_int[sz_iw];
      w = new double[sz_w];
      for(int i = 0; i < sz_w; i++){
        w[i] = 0;
      }

      x_val = new double[nnz];
      res0 = new double[nnz_out];
      for(int i = 0; i < nnz; i++){
        x_val[i] = 0;
        res0[i] = 0;
      }
      Logger::log() << Logger::Debug << "declared variables" << Logger::endl;
      Logger::log() << Logger::Info << "Exiting configuration hook" << Logger::endl;
      return true;
    }

    bool OCPComponent::startHook()
    {

        Logger::In in(this->getName());
        Logger::log() << Logger::Info << "Entering start hook" << Logger::endl;

        /* Function input and output */
        //parameters that need to be set a0, q_dot0, s0, s_dot0 TODO: read all from orocos ports
        Logger::log() << Logger::Debug << "Value of num joints = " << integer_props[0] << Logger::endl;
        vector<double> q0(integer_props[0],0);
        vector<double> q_dot0(integer_props[0],0);

        // Assign a fixed size and memory to the vectors associated with the ports
        m_q_actual.assign(integer_props[0], 0);
        m_qdot_actual.assign(integer_props[0], 0);
        m_q_command.assign(integer_props[0], 0);
        m_qd_command.assign(integer_props[0], 0);
        m_qdd_command.assign(integer_props[0], 0);

        if (inp_ports[0].read(m_q_actual) != NoData){
          Logger::log() << Logger::Debug << "Reading joint pos from robot_sim" << Logger::endl;
          for(int i = 0; i<integer_props[0]; i++){
            Logger::log() << Logger::Debug << "Initializing the joint values = " << m_q_actual[i] << Logger::endl;
            q0[i] = m_q_actual[i];
          }
        }
        else{
          Logger::log() << Logger::Error << "Failed to read robot joint positions" << Logger::endl;
          return false;
        }
        if (inp_ports[1].read(m_qdot_actual) != NoData){
          Logger::log() << Logger::Debug << "Read joint vel from robot_sim" << Logger::endl;
          for(int i = 0; i<integer_props[0]; i++){
            Logger::log() << Logger::Debug << "Initializing the joint vel values = " << m_qdot_actual[i] << Logger::endl;
            q_dot0[i] = m_qdot_actual[i];
          }
        }
        else{
          Logger::log() << Logger::Debug
          << "Failed to read robot velocities. Assigning zero values." << Logger::endl;
        }



        q0_start = js_prop["q0"]["start"].get<int>();
        q_start = js_prop["q"]["start"].get<int>();
        q_dot_start = js_prop["q_dot"]["start"].get<int>();
        q_ddot_start = js_prop["q_ddot"]["start"].get<int>();
        max_vel_loc = js_prop["max_vel"]["start"].get<int>();
        max_acc_loc = js_prop["max_acc"]["start"].get<int>();
        if(integer_props[1]){
          goal_start = js_prop["qdes"]["start"].get<int>();
        }
        else{
          goal_start = js_prop["fk_des"]["start"].get<int>();
        }

        //Initilializing the parameters to the correct values of x
        for(int i = 0; i < integer_props[0]; i++){
          x_val[q0_start + i] = q0[i];
          for(int j = 0; j < p_horizon + 1; j++){
            x_val[q_start + j*(integer_props[0]) + i] = q0[i];
          }
        }

        Logger::log() << Logger::Debug << "Initialized decision" << Logger::endl;

        //Adding the desired final joint position
        for(int i = 0; i < vector_props[0].size(); i++){
            x_val[goal_start + i] = vector_props[0][i];
          }

        x_val[max_acc_loc] = double_props[1]; //setting maximum acceleration.
        x_val[max_vel_loc] = double_props[0]; //setting maximum velocity

        // Allocate memory (thread-safe)
        Logger::log() << Logger::Debug << "Allocating memory" << Logger::endl;
        casadi_c_incref_id(f_id);

        /* Evaluate the function */
        arg[0] = x_val;
        res[0] = res0;
        Logger::log() << Logger::Debug << "Creating arguments for casadi function" << Logger::endl;

        // Checkout thread-local memory (not thread-safe)
        mem = casadi_c_checkout_id(f_id);

        // Evaluation is thread-safe TODO: add error handling if the OCP fails to find a solution
        Logger::log() << Logger::Debug << "Evaluating casadi OCP function" << Logger::endl;

        bool solver_status = casadi_c_eval_id(f_id, arg, res, iw, w, mem);
        Logger::log() << Logger::Error << "Solver status = " << solver_status << Logger::endl;
        if(solver_status){
          Logger::log() << Logger::Error << "OCP computation failed" << Logger::endl;
          return false;
        }

        Logger::log() << Logger::Info << "Exiting start hook" << Logger::endl;
        return true;
    }

    void OCPComponent::updateHook()
    {

      Logger::log() << Logger::Debug << "Entering UpdateHook" << Logger::endl;
      //Apply the control inputs
      //read values for q_command, q_d_command and q_dd_command
      if(sequence < p_horizon){
        for(i = 0; i < integer_props[0]; i++){
          m_q_command[i] = res0[i +  sequence*integer_props[0]];
          m_qd_command[i] = res0[q_dot_start + i + sequence*integer_props[0]];
          m_qdd_command[i] = res0[q_ddot_start + i + sequence*integer_props[0]];
        }
      }
      else if(sequence == p_horizon){
        for(i = 0; i < integer_props[0]; i++){
          m_qd_command[i] = 0;
          m_qdd_command[i] = 0;
        }
        Logger::log() << Logger::Debug << "OCP finished: writing event" << Logger::endl;
        Logger::log() << Logger::Debug << p_ocp_fun + "_done" << Logger::endl;
        port_eout.write(p_ocp_fun + "_done");
      }
      //Write the q, qd and qdd commands into the respective ports
      out_ports[0].write(m_q_command);
      // Logger::log() << Logger::Debug << "Writing into velocity port for sequence " << sequence <<  Logger::endl;
      out_ports[1].write(m_qd_command);
      out_ports[2].write(m_qdd_command);
      sequence++;
      this->trigger(); //TODO: shouldn't this trigger be removed?
    }

    void OCPComponent::stopHook()
    {
      Logger::log() << Logger::Debug << "Entering the stopHook" << Logger::endl;
      sequence = 0;
    }

    void OCPComponent::cleanupHook()
    {
      Logger::log() << Logger::Debug << "Entering the cleanupHook" << Logger::endl;
      casadi_c_decref_id(f_id);
      casadi_c_pop();
      free(x_val);
      free(res0);
      free(w);
      Logger::log() << Logger::Debug << "Exiting the cleanupHook" << Logger::endl;

    }

ORO_CREATE_COMPONENT(OCPComponent)
