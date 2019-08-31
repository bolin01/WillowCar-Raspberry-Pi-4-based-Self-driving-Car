#include <mpc_traj_follower/perception.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "perception_node");
    ros::NodeHandle nh;

    mpc_traj_follower::Perception perception_node(nh);

    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}