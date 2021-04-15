#include "MPCComponent.hpp"

#define M_PI 3.14159265358979323846
#define TIMEOUT 500 // [ms]


    MPCComponent::MPCComponent(const string &name) : TaskContext(name, PreOperational), p_numjoints(14), horizon(15), degrees_to_radians(M_PI / 180.0),  mpc_rate(20), time(0.0), wait(true)
    {
        //Adding properties
        this->addProperty("mpc_rate", mpc_rate).doc("Control frequency of MPC");
        this->addProperty("horizon", horizon).doc("Horizon size of the MPC");
        this->addProperty("ocp_file", ocp_file).doc("The casadi file that will compute the OCP.");
        this->addProperty("mpc_file", mpc_file).doc("The casadi file that will compute the MPC.");
        this->addProperty("predict_file", predict_file).doc("The casadi file that will simulate the next state of MPC.");
        this->addProperty("shift_file", shift_file).doc("The casadi file that will shift the MPC horizon by one step.");
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


        m_joint_states.name.resize(p_numjoints);
        m_joint_states.position.resize(p_numjoints);

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
      Logger::In in(this->getName());
      Logger::log() << Logger::Debug << "Got work ids" << Logger::endl;

          /* Allocate input/output buffers and work vectors*/

      res = new double*[sz_res];
      iw = new casadi_int[sz_iw];
      w = new double[70000];

      /* Function input and output */
      //parameters that need to be set a0, q_dot0, s0, s_dot0 TODO: read all from orocos ports
      double *q0 = new double[14];
      double *q_dot0 = new double[14];
      if (port_q_actual.read(m_q_actual) != NoData){
        printf("Read data!!!");
        for(int i = 0; i<14; i++){
          q0[i] = m_q_actual[i];
        }
      }
      else{
        double q0_def[14] = {-1.36542319,
              -0.74822507,
              2.05658987,
              0.52732208,
              2.4950726,
              -0.93756902,
              -1.71694542,
              1.32087,
              -0.77865726,
              -2.04601662,
              0.65292945,
              -2.25832585,
              -0.81930464,
              1.00047389}; //Defining default values. TODO: remove
        for(int i = 0; i<14; i++){
          q0[i] = q0_def[i];
        }
      }
      if (port_qdot_actual.read(m_qdot_actual) != NoData){
        for(int i = 0; i<14; i++){
          q_dot0[i] = m_qdot_actual[i];
        }
      }
      else{
        double q_dot0_def[14] = {0,0,0,0,0,0,0,
          0,0,0,0,0,0,0};
        q_dot0 = q_dot0_def;
      }
      double s0[1] = {0};
      double s_dot0[1] = {0};



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

      for(int i = 0; i < 28; i++){
        printf("%f\n",x_val[i]);
      }

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

      // Load the MPC function if different from the ocp Function
      if (ocp_file != mpc_file){
        // Clear the OCP file from memory
        /* Free memory (thread-safe) */
        casadi_c_decref_id(f_id);
        // Clear the last loaded Function(s) from the stack
        casadi_c_pop();

        //Load the mpc file into memory
        f_ret = casadi_c_push_file(mpc_file.c_str());
        if (f_ret) {
          cout << "Failed to load the mpc file " + mpc_file;
          return -1;
        }

        f_id = casadi_c_id("mpc_fun");
        n_in = casadi_c_n_in_id(f_id);
        n_out = casadi_c_n_out_id(f_id);

        sz_arg=n_in; sz_res=n_out; sz_iw=0; sz_w=0;

        casadi_c_work_id(f_id, &sz_arg, &sz_res, &sz_iw, &sz_w);
        printf("Work vector sizes:\n");
        printf("sz_arg = %lld, sz_res = %lld, sz_iw = %lld, sz_w = %lld\n\n",
            sz_arg, sz_res, sz_iw, sz_w);
        Logger::In in(this->getName());
        Logger::log() << Logger::Debug << "Got work ids for the MPC function" << Logger::endl;

        //Reallocating the space for the work-vector (which may differ with the algorithm)
        // delete [] w;
        // w = new double[sz_w];
        casadi_c_incref_id(f_id);
        mem = casadi_c_checkout_id(f_id);

      }

      Logger::log() << Logger::Debug << "Exiting configuration hook" << Logger::endl;
      return true;
    }

    bool MPCComponent::startHook()
    {

            Logger::log() << Logger::Debug << "Entering startHook" << Logger::endl;
            Logger::In in(this->getName());


        return true;
    }

    void MPCComponent::updateHook()
    {

        Logger::log() << Logger::Debug << "Entering updateHook" << Logger::endl;
        if (port_q_actual.read(m_q_actual) == NoData){
          Logger::log() << Logger::Error << "joint position input port in MPC read no data " << Logger::endl;
        }
        if (port_qdot_actual.read(m_qdot_actual) == NoData){
          Logger::log() << Logger::Error << "joint velocity input port in MPC read no data " << Logger::endl;
        }
        //Apply the control inputs
        //Implementing warm-starting. Hardcoded now, will shift to using the json thing.
        //warm-starting  the states
        for(int i = 0; i<14; i++){
          for(int j = 0; j<14;j++){
            if(i < 13){
              x_val[i*14 + j] = res0[(i+1)*14 + j]; //warmstarting q
              x_val[196 + i*14 + j] = res0[196 + (i+1)*14 + j]; //warmstarting q_dot
            }
            else{ //initializing at the last stage
              x_val[i*14 + j] = res0[(i)*14 + j]; //warmstarting q
              x_val[196 + i*14 + j] = res0[196 + (i)*14 + j]; //warmstarting q_dot
            }
          }
          if (i < 13){
          x_val[392 + i] = res0[392 + i + 1]; //warmstart s
          x_val[406 + i] = res0[406 + i + 1]; //warmstart s_dot
          }
          else{
            x_val[392 + i] = res0[392 + i]; //warmstart s
            x_val[406 + i] = res0[406 + i]; //warmstart s_dot
          }
        }

        //warm-starting the control variables
        for(int i = 0; i<13; i++){
          for(int j = 0; j<14;j++){
            if(i < 12){
            x_val[420 + i*14 + j] = res0[420 + (i+1)*14 + j]; //warmstarting q_ddot
            }
            else{
              x_val[420 + i*14 + j] = res0[420 + (i)*14 + j];
            }
          }
          for(int j = 0; j<2;j++){
            if(i < 12){
            x_val[615 + i*2 + j] = res0[615 + (i+1)*2 + j]; //warmstarting slack_1
            }
            else{
              x_val[615 + i*2 + j] = res0[615 + (i)*2 + j]; //warmstarting slack_1
            }
          }
          if(i < 12){
            x_val[602 + i] = res0[602 + i + 1]; //warmstart s_ddot
            x_val[641 + i] = res0[641 + i + 1]; //warmstart slack_2
          }
          else{
            x_val[602 + i] = res0[602 + i]; //warmstart s_ddot
            x_val[641 + i] = res0[641 + i]; //warmstart slack_2
          }
        }

        //Initializing the starting parameters
        for(int i = 0; i<14; i++){
          x_val[654 + i] = x_val[i];
          x_val[668 + i] = x_val[196 + i];
          x_val[682] = x_val[392];
          x_val[683] = x_val[406];
        }

        printf("s value is : %f", x_val[682]);


        // if(flag == true){
        // arg[0] = res0;
        // res[0] = x_val;
        // flag = false;
        // }
        // else{
        //   arg[0] = x_val;
        //   res[0] = res0;
        //   flag = true;
        // }
        casadi_c_eval_id(f_id, arg, res, iw, w, mem);
        this->trigger(); //TODO: shouldn't this trigger be removed?
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

    void predictFunction(){
      // Code to predict the future states based on the dynamics function
    }

    void shift(){
      // Code to shift x_vals for proper warm-starting of the MPC
    }

ORO_CREATE_COMPONENT(MPCComponent)
