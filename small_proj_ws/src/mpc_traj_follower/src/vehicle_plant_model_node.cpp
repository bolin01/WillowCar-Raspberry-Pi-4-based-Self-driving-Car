#include <mpc_traj_follower/vehicle_plant_model.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_plant_model_node");
    ros::NodeHandle nh;

    mpc_traj_follower::VehiclePlantModel vehicle_plant_model_node(nh);
    ros::Rate loop_rate(1);

    // This is for testing only. We'll not call publishVehicleMsg explicitly when this node is finished.
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}