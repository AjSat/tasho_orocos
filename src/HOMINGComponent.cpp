#include "HOMINGComponent.hpp"

#define M_PI 3.14159265358979323846
#define TIMEOUT 500 // [ms]


    HOMINGComponent::HOMINGComponent(const string &name) : TaskContext(name, PreOperational), p_numjoints(14), horizon(15), degrees_to_radians(M_PI / 180.0),  mpc_rate(20), time(0.0), wait(true)
    {
        //Adding properties
        this->addProperty("mpc_rate", mpc_rate).doc("Control frequency of MPC");
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

    HOMINGComponent::~HOMINGComponent()
    {
    }

    bool HOMINGComponent::configureHook()
    {


            Logger::In in(this->getName());
            Logger::log() << Logger::Debug << "Entering configuration hook" << Logger::endl;
        return true;
    }

    bool HOMINGComponent::startHook()
    {

            Logger::log() << Logger::Debug << "Entering startHook" << Logger::endl;
            Logger::In in(this->getName());

        return true;
    }

    void HOMINGComponent::updateHook()
    {

        Logger::log() << Logger::Debug << "Entering updateHook" << Logger::endl;
        this->trigger(); //TODO: shouldn't this trigger be removed?
    }

    void HOMINGComponent::stopHook()
    {
    }

    void HOMINGComponent::cleanupHook()
    {

    }

ORO_CREATE_COMPONENT(HOMINGComponent)
