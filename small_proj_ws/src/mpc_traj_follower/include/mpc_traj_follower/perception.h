/*
 *
 */

#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <ros/ros.h>
#include <vector>
#include <hkj_msgs/VehicleState.h>
#include <hkj_msgs/RoadConditionVector.h>
#include <hkj_msgs/RoadCondition.h>
#include <iostream>
#include <climits>
#include <mutex>
#include <std_msgs/Float32.h>

namespace mpc_traj_follower {

struct Obstacle {
  public:
    float pos_x[4] = {0.0, 0.0, 0.0, 0.0};
    float pos_y[4] = {0.0, 0.0, 0.0, 0.0};
};

class Perception {
  public:
    Perception(ros::NodeHandle& nh);
    ~Perception();

    /** [Objective]
     * read offline roadmap info from CSV file
     * */
    void readRoadmapFromCSV();

    /** [Objective]
     * prepare perception msg based on state feedback from plant_model_node
     * return true if perception msg is correctly prepared
     * */
    hkj_msgs::RoadConditionVector preparePerceptionMsg(const hkj_msgs::VehicleState::ConstPtr& msg);
    
    void publishPerceptionMsg();

  private:
    /* ROS */
    ros::NodeHandle nh_;
    ros::Subscriber sub_cur_state_;                  // receive from plant_model_node
    ros::Publisher  pub_road_cond_;                  // publish to pnc_node

    /* Variable */
    // perception
    std::vector<std::vector<float>> waypoints_;      // an offline map of road bound waypoints read from csv
    std::vector<Obstacle> obs_rects_;                // an offline map of obstacle rectangles read from csv
    void parseRoadMap(const std::string& roadmap);   // Read waypoints from roadmap file
    void parseRoadMapLine(const std::string& line);  // Helper function called in parseRoadMap

    // vehicle state
    int getNextWaypointIndex(double x, double y);             // Return the closest waypoint to vehicle current position
    float distanceToVehicle(int index, double x, double y);   // Helper function called in getNextWaypointIndex
    float distance(float x1, float y1, float x2, float y2);   // Helper function called in distanceToVehicle
    std::vector<std::vector<float>> getWaypoints(int index);  // Return the next few waypoints that the vehicle can see

    void vehicleStateCallback(const hkj_msgs::VehicleState::ConstPtr& msg);
};


} /* end of namespace mpc_traj_follower */


#endif /* PNC_H */