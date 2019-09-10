#include <mpc_traj_follower/vehicle_plant_model.h>

namespace mpc_traj_follower {

VehiclePlantModel::VehiclePlantModel(ros::NodeHandle& nh) : nh_(nh)
{   
    /* ROS */
    // sub_actuation_ = nh_.subscribe("/actuation", 10, &VehiclePlantModel::actuationCallback, this);     // receive from pnc_node
    // pub_cur_state_ = nh_.advertise<hkj_msgs::VehicleState>("/vehicle_states", 10);   // publish to pnc_node and perception_node
    
    // For testing, subscribe to perception node
    sub_actuation_ = nh_.subscribe("/vehicle_actuator", 10, &VehiclePlantModel::actuationCallback, this);
    pub_cur_state_ = nh_.advertise<hkj_msgs::VehicleState>("/vehicle_states", 10);   // publish perception_node

    state_pos_x_    =  287;
    state_pos_y_    = -176;
    state_vel_x_    =  5;
    state_vel_y_    =  0;
    state_yaw_ang_  =  1.90;
    state_yaw_rate_ =  0;
    state_time_     =  0;
    nh_.getParam("integration_dt", dt_);
    
    // Initialize the car model
    car = Bicycle6();

    states_traj_.push_back(std::vector<float>(6, 0.0)); // push a 6-element all-zero vector into states_traj_
    traj_vt_size_ = 1;

    while (pub_cur_state_.getNumSubscribers() < 2)
        continue;
    ROS_INFO("Initialized vehicle_plant_model_node!");
    
    publishVehicleMsg(state_pos_x_, state_pos_y_, state_vel_x_, state_vel_y_, state_yaw_ang_, state_yaw_rate_);
}

VehiclePlantModel::~VehiclePlantModel() {}

void VehiclePlantModel::actuationCallback(const hkj_msgs::VehicleActuator::ConstPtr& msg)
{
    // Integrate
    Integrate(msg->steer_angle, msg->applied_force, state_time_, state_time_ + dt_);
    state_time_ += dt_;

    publishVehicleMsg(state_pos_x_, state_pos_y_, state_vel_x_, state_vel_y_, state_yaw_ang_, state_yaw_rate_);
}

void VehiclePlantModel::publishVehicleMsg(float pos_x, float pos_y, float vel_x, float vel_y, float yaw_angle, float yaw_rate)
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
    ROS_INFO("Plant - Vehicle state published: x = %f, y = %f, yaw_angle = %f", pos_x, pos_y, yaw_angle);
}

void VehiclePlantModel::Integrate(std::vector<double> steers, std::vector<double> forces, double t0, double t1)
{
    std::vector<double> states;
    states.push_back(state_pos_x_);
    states.push_back(state_vel_x_);
    states.push_back(state_pos_y_);
    states.push_back(state_vel_y_);
    states.push_back(state_yaw_ang_);
    states.push_back(state_yaw_rate_);

    // Integration
    double step = car.setInput(steers, forces, t0, t1);
    boost::numeric::odeint::integrate(car, states, t0, t1, step);  // We can add an observer here for more outputs

    // update state variable
    state_pos_x_ = states[0];
    state_vel_x_ = states[1];
    state_pos_y_ = states[2];
    state_vel_y_ = states[3];
    state_yaw_ang_ = states[4];
    state_yaw_rate_ = states[5];
}

void VehiclePlantModel::setState(float pos_x, float pos_y, float vel_x, float vel_y, float yaw_angle, float yaw_rate)
{
    // Set the initial condition
    assert (state_time_ < 1e-6);

    state_pos_x_ = pos_x;
    state_vel_x_ = vel_x;
    state_pos_y_ = pos_y;
    state_vel_y_ = vel_y;
    state_yaw_ang_ = yaw_angle;
    state_yaw_rate_ = yaw_rate; 
}
} /* end of namespace mpc_traj_follower */