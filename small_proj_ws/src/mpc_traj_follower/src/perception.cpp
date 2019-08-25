#include <mpc_traj_follower/perception.h>

namespace mpc_traj_follower {

Perception::Perception(ros::NodeHandle& nh) : nh_(nh)
{
    /* ROS */
    sub_cur_state_ = nh_.subscribe("/vehicle_states", 10, &Perception::vehicleStateCallback, this);
    pub_road_cond_ = nh_.advertise<hkj_msgs::RoadConditionVector>("/perception", 10);

    ROS_INFO("Initialized perception_node!");
}

Perception::~Perception() {}

void Perception::vehicleStateCallback(const hkj_msgs::VehicleState::ConstPtr& msg)
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

bool Perception::readRoadmapFromCSV()
{
    /** TODO List: 
     *    1. read roadmap data (waypoints + obstacle) from a CSV file
     *    2. store the read information above into class member variables
     *    3. return true/flase (maybe set a flag indicating if perception prepared correctly)
     * */ 
}

bool Perception::preparePerceptionMsg(double pos_x, double pos_y, double yaw_ang)
{
    /** TODO List: 
     *    1. find an apropriate range of road condition waypoints & obstacle sets
     *    2. store the found information above into class member variables
     *    3. return true/flase (maybe set a flag indicating if perception prepared correctly)
     * */ 
}

void Perception::publishPerceptionMsg()
{   
    /** TODO List: 
     *    1. Check if the perception info is prepared
     *    2. pack the perception info to be sent using a temp variable
     * */ 


    // Publish a dummy RoadConditionVector to test the pipeline
    hkj_msgs::ObstacleRect dummy_obstacle_1;
    dummy_obstacle_1.obstacle_rectangle_pos_x = {0.0, 1.0, 1.0, 0.0};
    dummy_obstacle_1.obstacle_rectangle_pos_y = {0.0, 0.0, 1.0, 1.0};
    hkj_msgs::ObstacleRect dummy_obstacle_2;
    dummy_obstacle_2.obstacle_rectangle_pos_x = {0.0, 1.0, 1.0, 0.0};
    dummy_obstacle_2.obstacle_rectangle_pos_y = {2.0, 2.0, 3.0, 3.0};

    hkj_msgs::RoadCondition dummy_road_condition_1;
    dummy_road_condition_1.left_wp  = {0.0, 0.0};
    dummy_road_condition_1.mid_wp   = {1.0, 0.0};
    dummy_road_condition_1.right_wp = {2.0, 0.0};
    hkj_msgs::RoadCondition dummy_road_condition_2;
    dummy_road_condition_2.left_wp  = {0.0, 4.0};
    dummy_road_condition_2.mid_wp   = {1.0, 4.0};
    dummy_road_condition_2.right_wp = {2.0, 4.0};
    hkj_msgs::RoadCondition dummy_road_condition_3;
    dummy_road_condition_3.left_wp  = {0.0, 8.0};
    dummy_road_condition_3.mid_wp   = {1.0, 8.0};
    dummy_road_condition_3.right_wp = {2.0, 8.0};

    hkj_msgs::RoadConditionVector dummy_perception_msg;
    dummy_perception_msg.obstacle_rectangles.push_back(dummy_obstacle_1);
    dummy_perception_msg.obstacle_rectangles.push_back(dummy_obstacle_2);
    dummy_perception_msg.road_condition_vt.push_back(dummy_road_condition_1);
    dummy_perception_msg.road_condition_vt.push_back(dummy_road_condition_2);
    dummy_perception_msg.road_condition_vt.push_back(dummy_road_condition_3);

    pub_road_cond_.publish(dummy_perception_msg);
    ROS_INFO("Perception msg has been published!");
}

// clear previous stored perception data
void Perception::clearPerception()
{
    left_bound_wps_.clear();
    right_bound_wps_.clear();
    mid_line_wps_.clear();
}

// clear previous stored vehicle state data
void Perception::clearVehicleState()
{
    received_veh_state_.clear();
}

} /* end of namespace mpc_traj_follower */