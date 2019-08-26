#include <mpc_traj_follower/perception.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "perception_node");
    ros::NodeHandle nh;

    mpc_traj_follower::Perception perception_node(nh);

    // Explicitly control the perception node work at 40 Hz
    ros::Rate loop_rate(1);

    while ( ros::ok() )
    {
        perception_node.publishPerceptionMsg();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}