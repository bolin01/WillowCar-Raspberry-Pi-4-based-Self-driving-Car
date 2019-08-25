#include <mpc_traj_follower/pnc.h>

namespace mpc_traj_follower {

PNC::PNC(ros::NodeHandle& nh) : nh_(nh)
{   
    /* Variables */
    new_perception_ = false;
    new_veh_state_  = false;

    /* ROS */
    sub_road_cond_  = nh_.subscribe("/perception", 10, &PNC::perceptionCallback, this);
    sub_cur_state_  = nh_.subscribe("/vehicle_states", 10, &PNC::vehicleStatesCallback, this);
    pub_actuation_  = nh_.advertise<hkj_msgs::VehicleActuator>("/vehicle_actuator", 10);

    ROS_INFO("Initialized pnc_node!");
}

PNC::~PNC() {}

void PNC::perceptionCallback(const hkj_msgs::RoadConditionVector::ConstPtr& msg)
{   
    // clear stored perception data
    clearPerception();
    // set receiving new perception flag true
    new_perception_ = true;

    for (auto& elem : msg->road_condition_vt)
    {
        left_bound_wps_x_.push_back(elem.left_wp[0]);
        right_bound_wps_x_.push_back(elem.right_wp[0]);
        mid_line_wps_x_.push_back(elem.mid_wp[0]);

        left_bound_wps_y_.push_back(elem.left_wp[1]);
        right_bound_wps_y_.push_back(elem.right_wp[1]);
        mid_line_wps_y_.push_back(elem.mid_wp[1]);
    }

    for (auto& elem : msg->obstacle_rectangles)
    {   
        Obstacle obs;
        for (int i = 0; i < 4; i++) 
        {
            obs.pos_x[i] = elem.obstacle_rectangle_pos_x[i];
            obs.pos_y[i] = elem.obstacle_rectangle_pos_y[i];
        }
        obs_vec_.push_back(obs);
    }
}

void PNC::vehicleStatesCallback(const hkj_msgs::VehicleState::ConstPtr& msg)
{   
    // clear stored vehicle state data
    clearVehicleState();
    // set receiving new veh state flag true
    new_veh_state_ = true;

    received_veh_state_.push_back(msg->pos_x);
    received_veh_state_.push_back(msg->pos_y);
    received_veh_state_.push_back(msg->vel_x);
    received_veh_state_.push_back(msg->vel_y);
    received_veh_state_.push_back(msg->yaw_angle);
    received_veh_state_.push_back(msg->yaw_rate);
}

// CORE FUNCTION OF TRAJ OPTIMIZATION & MPC CONTROL
void PNC::processing()
{
    /** [Psudo code]
     * 
     *  Actuation(t + 1) = F( State(t), Perception(t + 1) );
     *  next_actuator = std::vector<float>{};
     *  for ( auto& elem : Actuation(t + 1) )
     *      next_actuator.push_back(elem);
     * 
     * */

    hkj_msgs::VehicleActuator next_veh_actuator;
    next_veh_actuator.applied_force = actuator_traj_.back()[0];
    next_veh_actuator.steer_angle   = actuator_traj_.back()[1];
}

// clear previous stored perception data
void PNC::clearPerception()
{
    left_bound_wps_x_.clear();
    left_bound_wps_y_.clear();
    right_bound_wps_x_.clear();
    right_bound_wps_y_.clear();
    mid_line_wps_x_.clear();
    mid_line_wps_y_.clear();
    obs_vec_.clear();
}

// clear previous stored vehicle state data
void PNC::clearVehicleState()
{
    received_veh_state_.clear();
}

} /* end of namespace mpc_traj_follower */