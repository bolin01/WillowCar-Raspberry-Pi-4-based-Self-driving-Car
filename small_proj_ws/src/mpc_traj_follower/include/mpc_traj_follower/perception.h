/*
 *
 */

#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <ros/ros.h>

namespace mpc_traj_follower {

class Perception {
  public:
    ros::Subscriber sub_cur_state_;         // receive from plant_model_node
    ros::Publisher  sub_road_cond_;         // publish to pnc_node
};


} /* end of namespace mpc_traj_follower */


#endif /* PNC_H */