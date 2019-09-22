/*
 *
 */

#ifndef KINEMATIC_PLANT_MODEL_H
#define KINEMATIC_PLANT_MODEL_H

#include <ros/ros.h>
#include <vector>
#include <hkj_msgs/VehicleState.h>
#include <hkj_msgs/VehicleActuator.h>
#include <Vehicle/BicycleKinematic.h>
#include <hkj_msgs/RoadConditionVector.h>  // For testing only... should be removed once pnc node is ready
#include <std_msgs/Float32.h>

namespace mpc_traj_follower {

class KinematicPlantModel {
  public:
    KinematicPlantModel(ros::NodeHandle& nh);
    ~KinematicPlantModel();

    // Publish vehicle state - It is made public only for testing purpose
    void publishVehicleMsg(float pos_x, float pos_y, float vel_x, float vel_y, float yaw_angle, float yaw_rate);
    void setState(float pos_x, float pos_y, float vel, float yaw_angle);
  
  private:
    /* ROS */
    ros::NodeHandle  nh_;                         // basic ROS NodeHandle
    ros::Subscriber  sub_actuation_;              // receive from pnc_node
    ros::Publisher   pub_cur_state_;              // publish to pnc_node and perception_node

    /* Variable */

    // states
    float state_x_;                               // state: global position x  [m]
    float state_y_;                               // state: global position y  [m]
    float state_v_;                               // state: global velocity x  [m/s]
    float state_yaw_ang_;                         // state: yaw andgle         [rad]
    float state_time_;                            // state: time               [s]
    float dt_;                                    // integration interval      [s]

    /* Call back that invokes integration */
    void actuationCallback(const hkj_msgs::VehicleActuator::ConstPtr& msg);

    // Vehicle plant model
    // TODO: expand it with a generic template when dealing with multiple vehicle models
    BicycleKinematic car;
    void Integrate(std::vector<double> steers, std::vector<double> acc, double t0, double t1);
};


} /* end of namespace mpc_traj_follower */

#endif /* PNC_H */