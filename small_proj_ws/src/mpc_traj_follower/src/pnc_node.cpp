#include <mpc_traj_follower/pnc.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pnc_node");
    ros::NodeHandle nh;

    mpc_traj_follower::PNC pnc_node(nh);
    
    ros::spin();

    return 0;
}