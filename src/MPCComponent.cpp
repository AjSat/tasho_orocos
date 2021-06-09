#include "MPCComponent.hpp"

#define M_PI 3.14159265358979323846
#define TIMEOUT 500 // [ms]


    MPCComponent::MPCComponent(const string &name) : TaskContext(name, PreOperational), degrees_to_radians(M_PI / 180.0), time(0.0), wait(true)
    {
        //Adding properties
        this->addProperty("js_prop_file", p_js_prop_file).doc("The location to the json file containing all the info pertaining to MPC.");
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

    MPCComponent::~MPCComponent()
    {

    }

    bool MPCComponent::configureHook()
    {

      //Adding properties
      FILE * pFile;
      pFile = fopen (p_js_prop_file.c_str(), "r");
      if (pFile == NULL) printf("Error opening file");
      jsp = json::parse(pFile);

      p_horizon = jsp["horizon"].get<int>();
      p_ocp_file = jsp["ocp_file"].get<std::string>();
      p_mpc_file = jsp["mpc_file"].get<std::string>();
      p_predict_file = jsp["pred_file"].get<std::string>();
      p_term_cond_pos = jsp["term_cond_pos"].get<int>();
      num_states = jsp["num_states"].get<int>();
      num_controls = jsp["num_controls"].get<int>();

      //Adding Properties
      vector_props = new vector<double>[jsp["num_props"].get<int>()];
      for(int i = 0; i < jsp["num_props"].get<int>(); i++){
        this->addProperty(jsp["props"][i]["name"].get<std::string>(),
          vector_props[i]).doc(jsp["props"][i]["desc"].get<std::string>());
      }

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

      if (casadi_c_push_file(p_ocp_file.c_str())) {
        cout << "Failed to load the ocp file " + p_ocp_file;
        return -1;
      }
      // Identify a Function by name
      f_id = casadi_c_id(jsp["ocp_fun_name"].get<std::string>().c_str());
      n_in = casadi_c_n_in_id(f_id);
      n_out = casadi_c_n_out_id(f_id);
      const casadi_int *sp_i;
      sp_i = casadi_c_sparsity_in_id(f_id, 0);
      casadi_int nrow = *sp_i++; /* Number of rows */
      casadi_int ncol = *sp_i++; /* Number of columns */
      nnz = sp_i[ncol]; /* Number of nonzeros */
      sp_i = casadi_c_sparsity_out_id(f_id, 0);
      nrow = *sp_i++; /* Number of rows */
      ncol = *sp_i++; /* Number of columns */
      nnz_out = sp_i[ncol]; /* Number of nonzeros */
      sz_arg=n_in; sz_res=n_out; sz_iw=0; sz_w=0;

      casadi_c_work_id(f_id, &sz_arg, &sz_res, &sz_iw, &sz_w);
      printf("Work vector sizes:\n");
      printf("sz_arg = %lld, sz_res = %lld, sz_iw = %lld, sz_w = %lld\n\n",
          sz_arg, sz_res, sz_iw, sz_w);
      Logger::In in(this->getName());
      Logger::log() << Logger::Debug << "Got work ids" << Logger::endl;

      /* Allocate input/output buffers and work vectors*/

      res = new double*[sz_res];
      arg = new const double*[sz_arg];
      iw = new casadi_int[sz_iw];
      w = new double[sz_w];

      x_val = new double[nnz];
      res0 = new double[nnz_out];
      for(int i = 0; i < nnz; i++){
        x_val[i] = 0;
        res0[i] = 0;
      }

      //load a prediction function into memory to simulate dynamics
      if (casadi_c_push_file(p_predict_file.c_str())) {
        cout << "Failed to load the mpc file " + p_predict_file;
        return -1;
      }

      pred_f_id = casadi_c_id(jsp["pred_fun_name"].get<std::string>().c_str());
      casadi_int sz_argp, sz_resp, sz_iwp, sz_wp;
      sz_argp = casadi_c_n_in_id(pred_f_id);
      sz_resp = casadi_c_n_out_id(pred_f_id);
      casadi_c_work_id(pred_f_id, &sz_argp, &sz_resp, &sz_iwp, &sz_wp);
      printf("Work vector sizes:\n");
      printf("sz_arg = %lld, sz_res = %lld, sz_iw = %lld, sz_w = %lld\n\n",
          sz_argp, sz_resp, sz_iwp, sz_wp);
      argp = new const double*[sz_argp];
      resp = new double*[sz_resp];
      iwp = new casadi_int[sz_iwp];
      wp = new double[sz_wp];
      argp[0] = x_val;
      argp[1] = res0;
      resp[0] = x_val;
      Logger::log() << Logger::Debug << "Allocating memory for pred fun" << Logger::endl;
      casadi_c_incref_id(pred_f_id);
      memp = casadi_c_checkout_id(pred_f_id);

      Logger::log() << Logger::Debug << "Exiting configuration hook" << Logger::endl;
      return true;

    }

    bool MPCComponent::startHook()
    {

      Logger::In in(this->getName());
      Logger::log() << Logger::Info << "Entering start hook" << Logger::endl;

      // Read messages from the input ports
      for(int i = 0; i < num_inp_ports; i++){
        if (inp_ports[i].read(m_inp_ports[i]) != NoData){
          Logger::log() << Logger::Debug << "Reading data from port " <<
            jsp["inp_ports"][i]["name"].get<std::string>() << Logger::endl;
          //assign the read messages to the OCP parameters
          for(int j = 0; j < m_inp_ports[i].size(); j++)
            x_val[jsp[jsp["inp_ports"][i]["var"].get<std::string>()]
              ["start"].get<int>() + j] = m_inp_ports[i][j];


          // warm start the states as well if specified
          if (jsp["inp_ports"][i]["warm_start"].get<int>()){
            std::string wvar = jsp["inp_ports"][i]["wvar"].get<std::string>();
            int var_start = jsp[wvar]["start"].get<int>();
            for (int j = 0; j < p_horizon + 1; j++){
              for (int k = 0; k < m_inp_ports[i].size(); k++){
                x_val[var_start + j*jsp[wvar]["jump"].get<int>() + k] = m_inp_ports[i][k];
              }
            }
          }
        }
      }

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
      casadi_c_eval_id(f_id, arg, res, iw, w, mem);

      // Load the MPC function if different from the ocp Function
      if (p_ocp_file != p_mpc_file){
        // Clear the OCP file from memory
        /* Free memory (thread-safe) */
        casadi_c_decref_id(f_id);
        // Clear the last loaded Function(s) from the stack
        // casadi_c_pop();

        //Load the mpc file into memory
        if (casadi_c_push_file(p_mpc_file.c_str())) {
          cout << "Failed to load the mpc file " + p_mpc_file;
          return -1;
        }

        f_id = casadi_c_id(jsp["mpc_fun_name"].get<std::string>().c_str());
        n_in = casadi_c_n_in_id(f_id);
        n_out = casadi_c_n_out_id(f_id);

        sz_arg=n_in; sz_res=n_out; sz_iw=0; sz_w=0;

        casadi_c_work_id(f_id, &sz_arg, &sz_res, &sz_iw, &sz_w);
        printf("Work vector sizes:\n");
        printf("sz_arg = %lld, sz_res = %lld, sz_iw = %lld, sz_w = %lld\n\n",
            sz_arg, sz_res, sz_iw, sz_w);
        res = new double*[sz_res];
        arg = new const double*[sz_arg];
        arg[0] = x_val;
        res[0] = res0;
        iw = new casadi_int[sz_iw];
        w = new double[sz_w];
        Logger::In in(this->getName());
        Logger::log() << Logger::Debug << "Got work ids for the MPC function" << Logger::endl;
        //Reallocating the space for the work-vector (which may differ with the algorithm)
        casadi_c_incref_id(f_id);
        mem = casadi_c_checkout_id(f_id);
        Logger::log() << Logger::Debug << "Evaluating mpc fun :" << Logger::endl;
        casadi_c_eval_id(f_id, arg, res, iw, w, mem);
        Logger::log() << Logger::Debug << "Evaluated mpc fun :" << Logger::endl;

      }
      Logger::log() << Logger::Debug << "Exiting start hook" << Logger::endl;
      return true;
    }

    void MPCComponent::updateHook()
    {

      Logger::log() << Logger::Debug << "Entering updateHook" << Logger::endl;

      //Apply the control inputs

      port_writer();
      if(!terminated){

        //shift the states
        for (int k = 0; k < num_states; k++){
          std::string state_name = jsp["states"][k].get<std::string>();
          int var_start = jsp[state_name]["start"].get<int>();
          int var_size = jsp[state_name]["size"].get<int>();
          int var_jump = jsp[state_name]["jump"].get<int>();
          for(int i = 0; i < p_horizon; i++){
            for(int j = 0; j < var_size; j++){
              x_val[var_start + i*var_jump + j] = res0[var_start + (i+1)*var_jump + j];
            }
          }
          int i = p_horizon;
          for(int j = 0; j<14;j++){
            x_val[var_start + i*var_jump + j] = res0[var_start + i*var_jump + j];
          }
        }

        //shifting controls
        for (int k = 0; k < num_controls; k++){
          std::string control_name = jsp["controls"][k].get<std::string>();
          int var_start = jsp[control_name]["start"].get<int>();
          int var_size = jsp[control_name]["size"].get<int>();
          int var_jump = jsp[control_name]["jump"].get<int>();
          for(int i = 0; i < p_horizon-1; i++){
            for(int j = 0; j < var_size; j++){
              x_val[var_start + i*var_jump + j] = res0[var_start + (i+1)*var_jump + j];
            }
          }
          int i = p_horizon-1;
          for(int j = 0; j<14;j++){
            x_val[var_start + i*var_jump + j] = res0[var_start + i*var_jump + j];
          }
        }

        // Reading the sensor messages and updating the parameters
        // Read messages from the input ports and assign to parameters
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


        Logger::log() << Logger::Debug << "Evaluating dynamics :" << Logger::endl;
        casadi_c_eval_id(pred_f_id, argp, resp, iwp, wp, memp);
        Logger::log() << Logger::Debug << "Finished dynamics evaluation." << Logger::endl;

        //Monitor if the termination criteria is reached
        if(x_val[p_term_cond_pos] >= 12.14){
          Logger::log() << Logger::Debug << "*** MPC termination criteria reached: writing event ***" << Logger::endl;
          port_eout.write(p_mpc_file + "_mpc_done");
          terminated = true;
          //TODO: set the acceleration to zero
        }
        Logger::log() << Logger::Debug << "Evaluating mpc fun :" << Logger::endl;
        casadi_c_eval_id(f_id, arg, res, iw, w, mem);
        Logger::log() << Logger::Debug << "Evaluated mpc fun :" << Logger::endl;
      }
    }

    void MPCComponent::stopHook()
    {
      Logger::log() << Logger::Debug << "Entering the stopHook" << Logger::endl;
    }

    void MPCComponent::cleanupHook()
    {
      Logger::log() << Logger::Debug << "Entering the cleanupHook" << Logger::endl;
      casadi_c_decref_id(f_id);
      // casadi_c_pop();
      Logger::log() << Logger::Debug << "Exiting the cleanupHook" << Logger::endl;
    }

    //A function to read the relevant values from xvals and write into ports
    void MPCComponent::port_writer(){
      // Reading the message to send to output ports
      for(int i = 0; i < num_out_ports; i++){
        out_port_var = jsp["out_ports"][i]["var"].get<std::string>();
        for(int j = 0; j < m_out_ports[i].size(); j++)
          m_out_ports[i][j] = res0[jsp[out_port_var]["start"].get<int>() + j];
      }
      //Writing messages to the output ports
      for(int i = 0; i < num_out_ports; i++) out_ports[i].write(m_out_ports[i]);
    }


ORO_CREATE_COMPONENT(MPCComponent)
