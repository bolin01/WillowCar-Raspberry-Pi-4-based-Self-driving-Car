/*
 *
 */

#ifndef PNC_H
#define PNC_H

#include <ros/ros.h>
#include <hkj_msgs/VehicleActuator.h>
#include <hkj_msgs/VehicleState.h>
#include <hkj_msgs/RoadCondition.h>
#include <hkj_msgs/ObstacleRect.h>
#include <hkj_msgs/RoadConditionVector.h>
#include <vector>

namespace mpc_traj_follower {

class PNC {
  public:
    PNC(ros::NodeHandle& nh);
    ~PNC();

    void perceptionCallback(const hkj_msgs::RoadConditionVector::ConstPtr& msg);
    virtual hkj_msgs::VehicleActuator prepareVehicleInput(const hkj_msgs::RoadConditionVector::ConstPtr& msg);  // Override this in derived classs

  private:
    /* ROS */
    ros::NodeHandle nh_;
    ros::Subscriber sub_;                   // receive from perception node
    ros::Publisher  pub_;                   // publish to vehicle_plant_node
    
};

} /* end of namespace mpc_traj_follower */

#endif /* PNC_H */