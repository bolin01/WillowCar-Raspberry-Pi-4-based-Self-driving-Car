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

struct Obstacle {
  public:
    float pos_x[4] = {0.0, 0.0, 0.0, 0.0};
    float pos_y[4] = {0.0, 0.0, 0.0, 0.0};
};

class PNC {
  public:
    PNC(ros::NodeHandle& nh);
    ~PNC();

  private:
    /* ROS */
    ros::NodeHandle nh_;
    ros::Subscriber sub_road_cond_;                   // receive from plant_model_node
    ros::Subscriber sub_cur_state_;                   // receive from plant_model_node
    ros::Publisher  pub_actuation_;                   // publish to vehicle_plant_node

    /* Variable */
    // perception
    bool new_perception_;                             // if a new perception signal received
    std::vector<float> left_bound_wps_x_;               // left bound waypoint x position vector
    std::vector<float> left_bound_wps_y_;               // left bound waypoint y position vector
    std::vector<float> right_bound_wps_x_;              // right bound waypoint x position vector
    std::vector<float> right_bound_wps_y_;              // right bound waypoint y position vector
    std::vector<float> mid_line_wps_x_;                 // middle line waypoint x position vector
    std::vector<float> mid_line_wps_y_;                 // middle line waypoint y position vector
    std::vector<Obstacle> obs_vec_;                   // obstacle sets vector
    // vehicle state
    bool new_veh_state_;                              // if a new vehicle state is received
    std::vector<float> received_veh_state_;           // vehicle state received from plant_model_node
    std::vector<std::vector<float>> actuator_traj_;   // trajectory of actuator
    std::vector<float> next_actuator_;                // actuator value to be sent to plant_model_node

    void perceptionCallback(const hkj_msgs::RoadConditionVector::ConstPtr& msg);
    void vehicleStatesCallback(const hkj_msgs::VehicleState::ConstPtr& msg);
    void clearPerception();
    void clearVehicleState();
    bool checkNewPerception() const { return new_perception_; }
    bool checkNewVehState() const  {return new_veh_state_; }
    bool preparedForProcessing() const;
    void processing();
    void publishActuation();
};

} /* end of namespace mpc_traj_follower */

#endif /* PNC_H */