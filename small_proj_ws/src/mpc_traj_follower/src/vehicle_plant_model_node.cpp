#include <mpc_traj_follower/vehicle_plant_model.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_plant_model_node");
    ros::NodeHandle nh;

    mpc_traj_follower::VehiclePlantModel vehicle_plant_model_node(nh);
    
    ros::spin();

    return 0;
}