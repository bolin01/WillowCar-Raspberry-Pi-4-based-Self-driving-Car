#include <mpc_traj_follower/kinematic_plant_model.h>

namespace mpc_traj_follower {

KinematicPlantModel::KinematicPlantModel(ros::NodeHandle& nh) : nh_(nh)
{   
    /* ROS */
    sub_actuation_ = nh_.subscribe("/vehicle_actuator", 10, &KinematicPlantModel::actuationCallback, this);
    pub_cur_state_ = nh_.advertise<hkj_msgs::VehicleState>("/vehicle_states", 10);   // publish perception_node

    // Get initial condition from parameters
    nh.getParam("pos_x", state_x_);
    nh.getParam("pos_y", state_y_);
    nh.getParam("vel", state_v_);
    nh.getParam("yaw", state_yaw_ang_);
    state_time_     =  0;
    nh_.getParam("integration_dt", dt_);
    
    // Initialize the car model
    car = BicycleKinematic();

    while (pub_cur_state_.getNumSubscribers() < 2)
        continue;
    ROS_INFO("Initialized vehicle_plant_model_node!");
    
    publishVehicleMsg(state_x_, state_y_, state_v_, 0.0, state_yaw_ang_, 0.0);
}

KinematicPlantModel::~KinematicPlantModel() {}

void KinematicPlantModel::actuationCallback(const hkj_msgs::VehicleActuator::ConstPtr& msg)
{
    // Integrate
    // Notice here the applied_force is actually the acceleration. We reuse the message of
    // dynamic model.
    Integrate(msg->steer_angle, msg->applied_force, state_time_, state_time_ + dt_);
    state_time_ += dt_;

    publishVehicleMsg(state_x_, state_y_, state_v_, 0.0, state_yaw_ang_, 0.0);
}

void KinematicPlantModel::publishVehicleMsg(float pos_x, float pos_y, float vel_x, float vel_y, float yaw_angle, float yaw_rate)
{   
    /** TODO List: 
     *    1. Check if the perception info is prepared
     *    2. pack the perception info to be sent using a temp variable
     * */ 
    hkj_msgs::VehicleState vstate;
    vstate.pos_x = pos_x;
    vstate.pos_y = pos_y;
    vstate.vel_x = vel_x;
    vstate.vel_y = vel_y;
    vstate.yaw_angle = yaw_angle;
    vstate.yaw_rate = yaw_rate;

    pub_cur_state_.publish(vstate);
    ROS_INFO("Plant - Vehicle state published: x = %f, y = %f, yaw_angle = %f, vel = %f", pos_x, pos_y, yaw_angle, vel_x);
}

void KinematicPlantModel::Integrate(std::vector<double> steers, std::vector<double> acc, double t0, double t1)
{
    std::vector<double> states;
    states.push_back(state_x_);
    states.push_back(state_y_);
    states.push_back(state_v_);
    states.push_back(state_yaw_ang_);

    // Integration
    double step = car.setInput(steers, acc, t0, t1);
    boost::numeric::odeint::integrate(car, states, t0, t1, step);  // We can add an observer here for more outputs

    // update state variable
    state_x_ = states[0];
    state_y_ = states[1];
    state_v_ = states[2];
    state_yaw_ang_ = states[3];
}

void KinematicPlantModel::setState(float pos_x, float pos_y, float vel, float yaw_angle)
{
    // Set the initial condition
    assert (state_time_ < 1e-6);

    state_x_ = pos_x;
    state_y_ = pos_y;
    state_v_ = vel;
    state_yaw_ang_ = yaw_angle;
}
} /* end of namespace mpc_traj_follower */