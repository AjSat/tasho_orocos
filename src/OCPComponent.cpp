#include "OCPComponent.hpp"

#define M_PI 3.14159265358979323846
#define TIMEOUT 500 // [ms]


    OCPComponent::OCPComponent(const string &name) : TaskContext(name), p_horizon(15), time(0.0), wait(true)
    {

      this->addProperty("js_prop_file", p_js_prop_file).doc("The location to the json file containing all the info pertaining to the OCP.");
      this->addPort("event_in", port_ein).doc("Events IN - eg supervisor");
      this->addPort("event_out", port_eout).doc("Events OUT - eg faults to supervisor");
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
      //Adding properties
      Logger::In in(this->getName());
      Logger::log() << Logger::Info << "Entering configuration hook" << Logger::endl;
      FILE * pFile;
      pFile = fopen (p_js_prop_file.c_str(), "r");
      if (pFile == NULL) printf("Error opening file");
      jsp = json::parse(pFile);

      p_horizon = jsp["horizon"].get<int>();
      double_props.resize(3, 0.0);
      integer_props.resize(2, 0); // = new int[2];
      vector_props = new vector<double>[2];
      this->addProperty("ocp_rate", double_props[2]).doc("Sampling rate of the OCP");
      this->addProperty("num_joints", integer_props[0]).doc("Number of joints");
      this->addProperty("goal_des", vector_props[0]).doc("Desired final pose of the robot arm.");
      this->addProperty("joint_pos", integer_props[1]).doc("Set to true if goal is in jointspace. False if Cartesian.");
      this->addProperty("max_vel", double_props[0]).doc("Maximum limits on joint velocities (rad/s)");
      this->addProperty("max_acc", double_props[1]).doc("Maximum limit on acceleration (rad/s^2)");

      //Adding ports
      /// Input
      num_inp_ports = jsp["num_inp_ports"].get<int>();
      inp_ports = new InputPort<vector<double>>[num_inp_ports];
      m_inp_ports = new vector<double>[num_inp_ports];
      for(int i = 0; i < num_inp_ports; i++){
        //Creating input ports
        this->addPort(jsp["inp_ports"][i]["name"].get<std::string>(),
          inp_ports[i]).doc(jsp["inp_ports"][i]["desc"].get<std::string>());
        // Assigning memory and size to the message vectors of the port
        m_inp_ports[i].resize(jsp[jsp["inp_ports"][i]["var"].get<std::string>()]
          ["size"].get<int>(), 0);
      }
      /// Output
      num_out_ports = jsp["num_out_ports"].get<int>();
      out_ports = new OutputPort<vector<double>>[num_out_ports];
      m_out_ports = new vector<double>[num_out_ports];
      for(int i = 0; i < num_out_ports; i++){
        //Creating output ports
        this->addPort(jsp["out_ports"][i]["name"].get<std::string>(),
          out_ports[i]).doc(jsp["out_ports"][i]["desc"].get<std::string>());
        // Assigning memory and size to the message vectors of the port
        m_out_ports[i].resize(jsp[jsp["out_ports"][i]["var"].get<std::string>()]
          ["size"].get<int>(), 0);
      }

      // Assign a fixed size and memory to the vectors associated with the ports

      f_ret = casadi_c_push_file(jsp["casadi_fun"].get<std::string>().c_str());
      Logger::log() << Logger::Info << "Loaded configuration file" << Logger::endl;
      if (f_ret) {
        cout << "Failed to load the ocp file " << jsp["casadi_fun"].get<std::string>();
        return -1;
      }
      // Identify a Function by name
      f_id = casadi_c_id(jsp["fun_name"].get<std::string>().c_str());
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
        sequence = 0;
        /* Function input and output */
        //parameters that need to be set a0, q_dot0, s0, s_dot0 TODO: read all from orocos ports
        Logger::log() << Logger::Debug << "Value of num joints = " << integer_props[0] << Logger::endl;
        vector<double> q0(integer_props[0],0);

        // Read messages from the input ports
        for(int i = 0; i < num_inp_ports; i++){
          if (inp_ports[i].read(m_inp_ports[i]) != NoData){
            Logger::log() << Logger::Debug << "Reading data from port " <<
              jsp["inp_ports"][i]["name"].get<std::string>() << Logger::endl;
            //assign the read messages to the OCP parameters
            for(int j = 0; j < m_inp_ports[i].size(); j++)
              x_val[jsp[jsp["inp_ports"][i]["var"].get<std::string>()]
                ["start"].get<int>() + j] = m_inp_ports[i][j];
          }
        }

        goal_start = jsp["goal_des"]["start"].get<int>();

        //Adding the desired final joint position
        for(int i = 0; i < vector_props[0].size(); i++){
            x_val[goal_start + i] = vector_props[0][i];
          }

        x_val[jsp["max_acc"]["start"].get<int>()] = double_props[1]; //setting maximum acceleration.
        x_val[jsp["max_vel"]["start"].get<int>()] = double_props[0]; //setting maximum velocity

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
      // Read the messages meant for the output ports
      std::string out_port_var;
      if(sequence < p_horizon){
        for(i = 0; i < num_out_ports; i++){
          out_port_var = jsp["out_ports"][i]["var"].get<std::string>();
          for(int j = 0; j < m_out_ports[i].size(); j++)
            m_out_ports[i][j] = res0[jsp[out_port_var]["start"].get<int>() + j +
              sequence*m_out_ports[i].size()];
        }
      }

      else if(sequence == p_horizon){
        for(i = 0; i < m_out_ports[2].size(); i++){
          m_out_ports[1][i] = 0;
          m_out_ports[2][i] = 0;
        }
        Logger::log() << Logger::Debug << "OCP finished: writing event" << Logger::endl;
        Logger::log() << Logger::Debug << jsp["fun_name"].get<std::string>() + "_done" << Logger::endl;
        port_eout.write(jsp["fun_name"].get<std::string>() + "_done");
      }

      //Writing messages to the output ports
      for(int i = 0; i < num_out_ports; i++) out_ports[i].write(m_out_ports[i]);
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
