/*
 *
 */

#ifndef VEHICLE_PLANT_MODEL_H
#define VEHICLE_PLANT_MODEL_H

#include <ros/ros.h>
#include <vector>
#include <hkj_msgs/VehicleState.h>
#include <hkj_msgs/VehicleActuator.h>
#include <Vehicle/Bicycle6.h>
#include <hkj_msgs/RoadConditionVector.h>  // For testing only... should be removed once pnc node is ready
#include <std_msgs/Float32.h>

namespace mpc_traj_follower {

class VehiclePlantModel {
  public:
    VehiclePlantModel(ros::NodeHandle& nh);
    ~VehiclePlantModel();

    // Publish vehicle state - It is made public only for testing purpose
    void publishVehicleMsg(float pos_x, float pos_y, float vel_x, float vel_y, float yaw_angle, float yaw_rate);
    void setState(float pos_x, float pos_y, float vel_x, float vel_y, float yaw_angle, float yaw_rate);
  
  private:
    /* ROS */
    ros::NodeHandle  nh_;                         // basic ROS NodeHandle
    ros::Subscriber  sub_actuation_;              // receive from pnc_node
    ros::Publisher   pub_cur_state_;              // publish to pnc_node and perception_node

    /* Variable */
    // actuator
    float act_force_;                             // actuator: force           [N]
    float act_steer_ang_;                         // actuator: steering angle  [rad]
    // states
    float state_pos_x_;                           // state: global position x  [m]
    float state_pos_y_;                           // state: global position y  [m]
    float state_vel_x_;                           // state: global velocity x  [m/s]
    float state_vel_y_;                           // state: global velocity y  [m/s]
    float state_yaw_ang_;                         // state: yaw andgle         [rad]
    float state_yaw_rate_;                        // state: yaw rate           [rad/s]
    float state_time_;                            // state: time               [s]
    float dt_;                                    // integration interval      [s]  
    // trajectory
    std::vector<std::vector<float>> states_traj_; // trajectory of states
    int traj_vt_size_;

    /* Params */
    void actuationCallback(const hkj_msgs::VehicleActuator::ConstPtr& msg);

    // Vehicle plant model
    // TODO: expand it with a generic template when dealing with multiple vehicle models
    Bicycle6 car;
    void Integrate(std::vector<double> steers, std::vector<double> forces, double t0, double t1);
};


} /* end of namespace mpc_traj_follower */

#endif /* PNC_H */