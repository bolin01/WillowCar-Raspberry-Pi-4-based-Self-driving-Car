#include <mpc_traj_follower/vehicle_plant_model.h>

namespace mpc_traj_follower {

VehiclePlantModel::VehiclePlantModel(ros::NodeHandle& nh) : nh_(nh)
{   
    /* ROS */
    sub_actuation_ = nh_.subscribe("/actuation", 10, &VehiclePlantModel::actuationCallback, this);     // receive from pnc_node
    pub_cur_state_ = nh_.advertise<hkj_msgs::VehicleState>("/vehicle_states", 10);   // publish to pnc_node and perception_node
    
    state_pos_x_ = 0.0;
    state_pos_y_ = 0.0;
    state_vel_x_ = 0.0;
    state_vel_y_ = 0.0;
    state_yaw_ang_ = 0.0;
    state_yaw_rate_ = 0.0;

    states_traj_.push_back( std::vector<float>(6, 0.0) ); // push a 6-element all-zero vector into states_traj_
    traj_vt_size_ = 1;

    ROS_INFO("Initialized vehicle_plant_model_node!");
}

VehiclePlantModel::~VehiclePlantModel() {}

void VehiclePlantModel::actuationCallback(const hkj_msgs::VehicleActuator::ConstPtr& msg)
{
    float applied_force = msg->applied_force;
    float steer_angle   = msg->steer_angle;
    
    /** [Psudo code]
     * 
     *  State(t + 1) = F( State(t), Actuation(t + 1) );
     *  next_state = std::vector<float>{};
     *  for ( auto& elem : State(t + 1) )
     *      next_state.push_back(elem);
     * 
     *  state_pos_x_   = State(t + 1)[0];
     *  state_pos_y_   = State(t + 1)[1];
     *  ......
     *  ......
     *  ......
     *  state_yaw_rate = State(t + 1)[5];
     * 
     * */

    std::vector<float> next_state;
    states_traj_.push_back(next_state);
    traj_vt_size_++;

    if (traj_vt_size_ == 0) 
        return;

    // parse info into next_veh_state and publish
    hkj_msgs::VehicleState next_veh_state;
    next_veh_state.pos_x      = states_traj_.back()[0];
    next_veh_state.pos_y      = states_traj_.back()[1];
    next_veh_state.vel_x      = states_traj_.back()[2];
    next_veh_state.vel_y      = states_traj_.back()[3];
    next_veh_state.yaw_angle  = states_traj_.back()[4];
    next_veh_state.yaw_rate   = states_traj_.back()[5];

    pub_cur_state_.publish(next_veh_state);
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



} /* end of namespace mpc_traj_follower */