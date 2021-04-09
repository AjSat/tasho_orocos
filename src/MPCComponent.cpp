#include "MPCComponent.hpp"
#include <casadi/casadi_c.h>

#define M_PI 3.14159265358979323846
#define TIMEOUT 500 // [ms]


    MPCComponent::MPCComponent(const string &name) : TaskContext(name, PreOperational), p_numjoints(14), horizon(15), degrees_to_radians(M_PI / 180.0),  mpc_rate(20), time(0.0), wait(true)
    {
        //Adding properties
        this->addProperty("mpc_rate", mpc_rate).doc("Control frequency of MPC");
        this->addProperty("ocp_file", ocp_file).doc("The casadi file that will compute the OCP.");
        this->addProperty("mpc_file", mpc_file).doc("The casadi file that will compute the MPC.");
        this->addProperty("predict_file", predict_file).doc("The casadi file that will simulate the next state of MPC.");
        this->addProperty("shift_file", shift_file).doc("The casadi file that will shift the MPC horizon by one step.");
        //Adding ports
        /// Input
        this->addPort("event_in", port_ein).doc("Events IN - eg supervisor");
        this->addPort("JointPositionCommand", port_joint_pos_command).doc("desired joint positions [rad]");
        this->addPort("JointVelocityCommand", port_joint_vel_command).doc("desired joint velocities [rad/s]");
        /// Output
        this->addPort("event_out", port_eout).doc("Events OUT - eg faults to supervisor");
        this->addPort("q_actual", port_q_actual).doc("current joint positions [rad]");
        this->addPort("qdot_actual", port_qdot_actual).doc("current joint velocities [rad/s]");


        //Fixed size
        m_joint_pos_command.positions.resize(p_numjoints);
        m_joint_vel_command.velocities.resize(p_numjoints);

        m_joint_pos_command.positions.assign(p_numjoints, 0);
        m_joint_vel_command.velocities.assign(p_numjoints, 0);


        m_joint_states.name.resize(p_numjoints);
        m_joint_states.position.resize(p_numjoints);

    }

    MPCComponent::~MPCComponent()
    {
    }

    bool MPCComponent::configureHook()
    {
      //Sanity check on integer types
      if (casadi_c_int_width()!=sizeof(casadi_int)) {
        printf("Mismatch in integer size\n");
        return -1;
      }
      if (casadi_c_real_width()!=sizeof(double)) {
        printf("Mismatch in double size\n");
        return -1;
      }

      int ret = casadi_c_push_file(ocp_file.c_str());
      if (ret) {
        cout << "Failed to load the ocp file " + ocp_file;
        return -1;
      }
      // ret = casadi_c_push_file(mpc_file.c_str());
      // if (ret) {
      //   cout << "Failed to load the mpc file " + mpc_file;
      //   return -1;
      // }

      // Identify a Function by name
      int id = casadi_c_id("ocp_fun");
      casadi_int n_in = casadi_c_n_in_id(id);
      casadi_int n_out = casadi_c_n_out_id(id);

      casadi_int sz_arg=n_in, sz_res=n_out, sz_iw=0, sz_w=0;

      casadi_c_work_id(id, &sz_arg, &sz_res, &sz_iw, &sz_w);
      printf("Work vector sizes:\n");
      printf("sz_arg = %lld, sz_res = %lld, sz_iw = %lld, sz_w = %lld\n\n",
          sz_arg, sz_res, sz_iw, sz_w);
      Logger::In in(this->getName());
      Logger::log() << Logger::Debug << "Got work ids" << Logger::endl;

          /* Allocate input/output buffers and work vectors*/
      const double *arg[sz_arg];
      double *res[sz_res];
      casadi_int iw[sz_iw];
      double w[sz_w];

      /* Function input and output */
      //parameters that need to be set a0, q_dot0, s0, s_dot0 TODO: read all from orocos ports
      double q0[14] = {-1.36542319,
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
            1.00047389}; //TODO: read from orocos port.
      double q_dot0[14] = {0,0,0,0,0,0,0,
      0,0,0,0,0,0,0};
      double s0[1] = {0};
      double s_dot0[1] = {0};



      double x_val[40000];
      double x_val2[40000];
      double res0[40000];
      double res2[40000];
      for(int i = 0; i < 40000; i++){
        x_val[i] = 0;
        x_val2[i] = 0;
        res0[i] = 0;
        res2[i] = 0;
      }

      //Initilializing the parameters to the correct values of x
      int h_size = 14;
      int q0_start = 654;
      int q_start = 0;
      int q_size = 14;
      for(int i = 0; i < q_size; i++){
        x_val[q0_start + i] = q0[i];
        for(int j = 0; j < h_size; j++){
          x_val[q_start + j*h_size + i] = q0[i];
        }
      }

      for(int i = 0; i < 28; i++){
        printf("%f\n",x_val[i]);
      }

      // Allocate memory (thread-safe)
      Logger::log() << Logger::Debug << "Allocating memory" << Logger::endl;
      casadi_c_incref_id(id);

      /* Evaluate the function */
      arg[0] = x_val;
      arg[1] = x_val2;
      res[0] = res0;
      res[1] = res2;
      Logger::log() << Logger::Debug << "Creating arguments for casadi function" << Logger::endl;
      // Checkout thread-local memory (not thread-safe)
      int mem = casadi_c_checkout_id(id);

      // Evaluation is thread-safe
      Logger::log() << Logger::Debug << "Evaluating casadi function" << Logger::endl;
      if (casadi_c_eval_id(id, arg, res, iw, w, mem)) return 1;


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
        this->trigger(); //TODO: shouldn't this trigger be removed?
    }

    void MPCComponent::stopHook()
    {
    }

    void MPCComponent::cleanupHook()
    {

    }

ORO_CREATE_COMPONENT(MPCComponent)
