#include <mpc_traj_follower/perception.h>

namespace mpc_traj_follower {

Perception::Perception(ros::NodeHandle& nh) : nh_(nh)
{
    // Preprocessing - read roadmap waypoints and obstacles
    readRoadmapFromCSV();

    /* ROS */
    sub_cur_state_ = nh_.subscribe("/vehicle_states", 10, &Perception::vehicleStateCallback, this);
    pub_road_cond_ = nh_.advertise<hkj_msgs::RoadConditionVector>("/perception", 10);

    while (pub_road_cond_.getNumSubscribers() < 1)
        continue;
    ROS_INFO("Perception - Initialized perception_node!");
}

Perception::~Perception() {}

void Perception::vehicleStateCallback(const hkj_msgs::VehicleState::ConstPtr& msg)
{   
    hkj_msgs::RoadConditionVector perception_msg = preparePerceptionMsg(msg);
    pub_road_cond_.publish(perception_msg);
    ROS_INFO("Perception - Perception msg has been published!");
}

void Perception::readRoadmapFromCSV()
{
    // Read offline roadmap from csv
    std::string roadmap_file;
    if (nh_.getParam("roadmap_file", roadmap_file))
    {
        parseRoadMap(roadmap_file);
    }
    else
        ROS_ERROR("Perception - Not able to find roadmap file.");

}

void Perception::parseRoadMap(const std::string& roadmap)
{
    // Parse the roadmap file and add waypoints
    std::istringstream rm_stream(roadmap);
    std::string line;
    std::getline(rm_stream, line);   // Ignore the first line (header)
    while (std::getline(rm_stream, line))
        parseRoadMapLine(line);
    ROS_INFO("Perception - Map parsed. We have %i waypoints", (int)waypoints_.size());
    assert (waypoints_.size());
}

void Perception::parseRoadMapLine(const std::string& line)
{
    // Parse each line in roadmap file and add waypoint vector
    std::istringstream line_stream(line);
    std::string number;
    // wp format: lb x,lb y,lcline x,lcline y,cline x,cline y,rcline x,rcline y,rb x,rb y,theta
    std::vector<float> wp;
    while (std::getline(line_stream, number, ','))
        wp.push_back(std::stof(number));
    waypoints_.push_back(wp);
}

float Perception::distance(float x1, float y1, float x2, float y2)
{
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

float Perception::distanceToVehicle(int index, double x, double y)
{
    // Compute the distance between vehicle and #index waypoint
    // We have three center lines; choose the smallest one
    assert (index >= 0 && index < waypoints_.size());
    float dist1 = distance(x, y, waypoints_[index][2], waypoints_[index][3]);
    float dist2 = distance(x, y, waypoints_[index][4], waypoints_[index][5]);
    float dist3 = distance(x, y, waypoints_[index][6], waypoints_[index][7]);
    return std::min({dist1, dist2, dist3});
}

int Perception::getNextWaypointIndex(double x, double y)
{
    int index = 0, min_dist = INT_MAX;
    for (size_t i=0; i<waypoints_.size(); ++i)
    {
        int dist = distanceToVehicle(i, x, y);
        if (dist < min_dist)
        {
            min_dist = dist;
            index = i;
        }
    }

    return index;
}

std::vector<std::vector<float>> Perception::getWaypoints(int index)
{
    double distance_can_see;
    nh_.getParam("distance_can_see", distance_can_see);
    double dist = 0.0;
    std::vector<std::vector<float>> wp;
    wp.push_back(waypoints_[index]);
    for (int i=index+1; i<waypoints_.size(); ++i)
    {
        const auto& wp0 = waypoints_[i-1];
        const auto& wp1 = waypoints_[i];
        dist += distance(wp0[4], wp0[5], wp1[4], wp1[5]);
        
        // Return all waypoints here if distance exceeds the maximum distance can see
        if (dist > distance_can_see)
            return wp;
        
        wp.push_back(wp1);
    }
    return wp;
}

hkj_msgs::RoadConditionVector Perception::preparePerceptionMsg(const hkj_msgs::VehicleState::ConstPtr& msg)
{
    /** TODO List: 
     *    1. find an apropriate range of road condition waypoints & obstacle sets
     *    2. store the found information above into class member variables
     *    3. return true/flase (maybe set a flag indicating if perception prepared correctly)
     * */

    // Get vehicle state from msg
    int index = getNextWaypointIndex(msg->pos_x, msg->pos_y);
    std::vector<std::vector<float>> wp = getWaypoints(index);
    ROS_INFO("Perception - Found %d waypoints!", int(wp.size()));
    hkj_msgs::RoadConditionVector perception_msg;

    for (int i=0; i<wp.size(); ++i)
    {
        hkj_msgs::RoadCondition road_condition;
        road_condition.lb_wp  = {wp[i][0], wp[i][1]};
        road_condition.lc_wp  = {wp[i][2], wp[i][3]};
        road_condition.c_wp   = {wp[i][4], wp[i][5]};
        road_condition.rc_wp  = {wp[i][6], wp[i][7]};
        road_condition.rb_wp  = {wp[i][8], wp[i][9]};
        perception_msg.road_condition_vt.push_back(road_condition);
    }

    // Add vehicle states
    perception_msg.pos_x = msg->pos_x;
    perception_msg.pos_y = msg->pos_y;
    perception_msg.vel_x = msg->vel_x;
    perception_msg.yaw_angle = msg->yaw_angle;
    perception_msg.yaw_rate = msg->yaw_rate;
    return perception_msg;
}
} /* end of namespace mpc_traj_follower */