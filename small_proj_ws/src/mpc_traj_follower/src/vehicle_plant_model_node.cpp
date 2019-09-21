#include <mpc_traj_follower/kinematic_plant_model.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_plant_model_node");
    ros::NodeHandle nh;

    mpc_traj_follower::KinematicPlantModel vehicle_plant_model_node(nh);
    ros::Rate loop_rate(2);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}