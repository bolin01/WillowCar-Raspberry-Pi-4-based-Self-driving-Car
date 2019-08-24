/*
 *
 */

#ifndef PNC_H
#define PNC_H

#include <ros/ros.h>
#include <hkj_msgs/VehicleActuator.h>
#include <hkj_msgs/VehicleState.h>

namespace mpc_traj_follower {

class PNC {
  public:
    ros::Subscriber sub_road_cond_;        // receive from plant_model_node
    ros::Subscriber sub_cur_state_;        // receive from plant_model_node
    ros::Publisher  pub_actuation_;        // publish to vehicle_plant_node
};


} /* end of namespace mpc_traj_follower */


#endif /* PNC_H */