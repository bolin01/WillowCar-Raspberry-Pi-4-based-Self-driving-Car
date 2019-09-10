#include <mpc_traj_follower/pnc.h>

namespace mpc_traj_follower {

PNC::PNC(ros::NodeHandle& nh) : nh_(nh)
{   
    /* ROS */
    sub_  = nh_.subscribe("/perception", 10, &PNC::perceptionCallback, this);
    pub_  = nh_.advertise<hkj_msgs::VehicleActuator>("/vehicle_actuator", 10);

    while (pub_.getNumSubscribers() < 1)
        continue;

    ROS_INFO("PNC - Initialized pnc_node!");
}

PNC::~PNC() {}

void PNC::perceptionCallback(const hkj_msgs::RoadConditionVector::ConstPtr& msg)
{   
    hkj_msgs::VehicleActuator act_msg = this->prepareVehicleInput(msg);
    pub_.publish(act_msg);
    ROS_INFO("PNC - Actuation input published.");
}

hkj_msgs::VehicleActuator PNC::prepareVehicleInput(const hkj_msgs::RoadConditionVector::ConstPtr& msg)
{
    hkj_msgs::VehicleActuator act_msg;
    
    for (int i=0; i<11; ++i)
    {
        act_msg.applied_force.push_back(0.0);
        act_msg.steer_angle.push_back(0.0);
    }

    return act_msg;
}

} /* end of namespace mpc_traj_follower */