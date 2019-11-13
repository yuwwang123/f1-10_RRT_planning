//Author: Yuwei Wang

#include "rrt/rrt.h"
#include <rrt/occupancy_grid.h>
#include <rrt/CSVReader.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

using namespace std;

const float MARGIN = 0.2;
const int MAX_ITER = 200;
const double X_SAMPLE_RANGE = 4.0;
const double Y_SAMPLE_RANGE = 4.0;
const double GOAL_THRESHOLD = 0.2;
const double STEER_RANGE = 0.5;
const float SCAN_RANGE = sqrt(pow(X_SAMPLE_RANGE,2.0)+ pow(Y_SAMPLE_RANGE, 2.0));
const float MAX_GOAL_AHEAD_DIST = min(X_SAMPLE_RANGE, Y_SAMPLE_RANGE) * 0.8;
const float LOOK_AHEAD_DIST = 0.5;
const float P_GAIN = 0.3;
const float SPEED = 2.0;

const string file_name = "/home/yuwei/rcws/logs/yuwei_wp.csv";

const string pose_topic = "odom";
const string scan_topic = "scan";
const string drive_topic = "nav";

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen_((std::random_device())()) {

    // ROS publishers & subscribers
    odom_sub_ = nh_.subscribe(pose_topic, 10, &RRT::odom_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);

    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
    obstacle_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("obstacles_inflated", 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>("path_found", 10);
    tree_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("rrt_tree", 10);
    pos_sp_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("pos_sp_", 10);
    std::uniform_real_distribution<double> distribution(-1.0,1.0);
    uni_dist_ = distribution;

    init_occupancy_grid();
    visualize_map();
    load_waypoints(file_name);
    reset_goal();
    ROS_INFO("Created new RRT Object.");
}

void RRT::init_occupancy_grid(){

    boost::shared_ptr<nav_msgs::OccupancyGrid const> map_ptr;
    map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(5.0));
    if (map_ptr == nullptr){ROS_INFO("No map received");}
    else{
        map_ = *map_ptr;
        map_updated_ = map_;
        ROS_INFO("Map received");
    }
    ROS_INFO("Initializing occupancy grid for map ...");
    occupancy_grid::inflate_obstacles(map_, MARGIN);
}

void RRT::visualize_map(){
    visualization_msgs::Marker dots;
    dots.header.stamp = ros::Time::now();
    dots.header.frame_id = "map";
    dots.id = 0;
    dots.ns = "obstacle";
    dots.type = visualization_msgs::Marker::POINTS;
    dots.scale.x = dots.scale.y = 0.04;
    dots.scale.z = 0.04;
    dots.action = visualization_msgs::Marker::ADD;
    dots.pose.orientation.w = 1.0;
    dots.color.r = 1.0;
    dots.color.a = 1.0;
    dots.lifetime = ros::Duration();

    for (int i=0; i<map_.data.size(); i++){
        if (int(map_.data.at(i))==100){
            geometry_msgs::Point p;
            p.x = occupancy_grid::ind2x(map_, i);
            p.y = occupancy_grid::ind2y(map_, i);
            p.z = 0.0;
            dots.points.push_back(p);
        }
    }
    obstacle_viz_pub_.publish(dots);
}

void RRT::load_waypoints(std::string file_name){
    waypoints_.clear();
    CSVReader reader(file_name);
    // Get the data from CSV File
    std::vector<std::vector<std::string> > dataList = reader.getData();
    // Print the content of row by row on screen
    for(std::vector<std::string> vec : dataList){
        geometry_msgs::Point wp;
        wp.x = std::stof(vec.at(0));
        wp.y = std::stof(vec.at(1));
        waypoints_.push_back(wp);
    }
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //
    using namespace occupancy_grid;
    tf::Quaternion q_tf;
    tf::quaternionMsgToTF(car_pose_.orientation, q_tf);
    tf::Vector3 origin;
    origin = tf::Vector3(car_pose_.position.x, car_pose_.position.y, 0.0);

    tf_.setOrigin(origin);
    tf_.setRotation(q_tf);

    map_updated_ = map_; // might be expensive to copy

    float angle_min = msg->angle_min;
    float angle_increment = msg->angle_increment;

    for (int i = 0; i < msg->ranges.size(); ++i) {
        float range = msg->ranges.at(i);
        if (range > SCAN_RANGE) {
            continue;
        }
        if (!isnan(range) && !isinf(range)) {
            float angle = angle_min + angle_increment * i;
            tf::Vector3 pos_in_car = tf::Vector3(range*cos(angle), range*sin(angle), 0.0);
            tf::Vector3 pos_in_map = tf_ * pos_in_car;
            if (!is_xy_occupied(map_, pos_in_map.x(), pos_in_map.y())){
                set_xy_occupied(map_updated_, pos_in_map.x(), pos_in_map.y());
                inflate_cell(map_updated_, xy2ind(map_updated_, pos_in_map.x(), pos_in_map.y()), 0.15);
            }
        }
    }
}

void RRT::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    //Get start
    car_pose_ = odom_msg->pose.pose;
    Node start_node;
    start_node.x = car_pose_.position.x;
    start_node.y = car_pose_.position.y;
    start_node.parent = 0;
    start_node.is_root = true;

    //Get Goal
    get_current_goal();

    std::vector<Node> tree;
    tree.push_back(start_node);
    Node latest_node = start_node;
    bool no_path = false;
    //  RRT main loop
    for (int i=0; i<MAX_ITER; i++){
        vector<double> sampled_point = sample();
        int nearest_ind = nearest(tree, sampled_point);
        Node new_node = steer(tree.at(nearest_ind), sampled_point);
        if (!check_collision(tree.at(nearest_ind), new_node)){
            new_node.is_root = false;
            new_node.parent = nearest_ind;
            tree.push_back(new_node);
            latest_node = new_node;
            if (is_goal(new_node, waypoints_.at(curr_goal_ind_).x, waypoints_.at(curr_goal_ind_).y)){
                ROS_INFO("Path found");
                break;
            }
        }
        if (i==MAX_ITER-1){
            no_path = true;
            ROS_INFO("Couldn't find a path");
        }
    };
    // construct path_found as Path message
    if (!no_path){
        vector<Node> path_found = find_path(tree, latest_node);
        nav_msgs::Path path;
        path.header.frame_id = "map";
     //   bool pos_sp_found = false;
      //  int pos_sp_ind = 0;
        for (int i=0; i<path_found.size(); i++){
            geometry_msgs::PoseStamped p;
            p.pose.position.x = path_found.at(i).x;
            p.pose.position.y = path_found.at(i).y;
            p.pose.orientation.w = 1.0;
            path.poses.push_back(p);
        }
        path_pub_.publish(path);
        track_path(path);
        visualize_tree(tree);
    }

}

float calculate_dist2(float x1, float x2, float y1, float y2){
    // considering that doing sqrt is expensive
    return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
}

void RRT::get_current_goal(){
    float dist_to_goal2 = calculate_dist2(waypoints_.at(curr_goal_ind_).x, car_pose_.position.x,
                              waypoints_.at(curr_goal_ind_).y,  car_pose_.position.y);
    // goal out of range, reset goal
    if (dist_to_goal2 > pow(MAX_GOAL_AHEAD_DIST, 2)){
        reset_goal();
    }
    // enough progress made, advance goal
    else if(dist_to_goal2 < pow(MAX_GOAL_AHEAD_DIST*0.7, 2)){
        advance_goal();
    }
}

void RRT::reset_goal(){
    boost::shared_ptr<nav_msgs::Odometry const> pose_ptr;
    pose_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>(pose_topic, ros::Duration(5.0));

    if(pose_ptr== nullptr){ROS_INFO("Failed to receive car's pose");}
    else{
        int closest_ind = find_closest_waypoint(waypoints_, pose_ptr->pose.pose);
        float closest_dist2 = calculate_dist2(waypoints_.at(closest_ind).x, pose_ptr->pose.pose.position.x,
                                              waypoints_.at(closest_ind).y, pose_ptr->pose.pose.position.y);
        if (closest_dist2 > pow(MAX_GOAL_AHEAD_DIST, 2)){
            throw "Couldn't find a goal in range. Reposition the car somewhere near the waypoints";
        }
        // advance from closest waypoint until one that is around 0.9 GOAL_AHEAD_DIST away
        curr_goal_ind_ = closest_ind;
        advance_goal();
    }
}

void RRT::advance_goal(){
    // advance in waypoints from current goal to a point that is around 0.9*MAX_GOAL_AHEAD distance ahead
    using namespace occupancy_grid;
    int curr_ind = curr_goal_ind_;
    if (curr_ind >= waypoints_.size()){ curr_ind = 0;}
    float pose_x = car_pose_.position.x;
    float pose_y = car_pose_.position.y;
    float curr_dist2 = calculate_dist2(waypoints_.at(curr_ind).x, pose_x, waypoints_.at(curr_ind).y, pose_y);

    while(curr_dist2 < pow(MAX_GOAL_AHEAD_DIST*0.9, 2)
      || is_xy_occupied(map_updated_, waypoints_.at(curr_ind).x, waypoints_.at(curr_ind).y)){
        curr_dist2 = calculate_dist2(waypoints_.at(curr_ind).x, pose_x, waypoints_.at(curr_ind).y, pose_y);
        curr_ind++;
        if (curr_ind >= waypoints_.size()) {curr_ind = 0;}
        if (curr_dist2 > pow(MAX_GOAL_AHEAD_DIST, 2)){break;}
    }
    curr_goal_ind_ = max(0, curr_ind-1);
}

int RRT::find_closest_waypoint(const vector<geometry_msgs::Point>& waypoints, const geometry_msgs::Pose& pose){
    float min_dist2 = 100000.0;
    int min_ind;
    for (int i=0; i<waypoints.size(); i++){
        float dist2 = calculate_dist2(waypoints.at(i).x, pose.position.x, waypoints.at(i).y, pose.position.y);
        if (dist2 < min_dist2){
            min_dist2 = dist2;
            min_ind = i;
        }
    }
    return min_ind;
}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space

    std::vector<double> sampled_point;
    double x = car_pose_.position.x + uni_dist_(gen_)*X_SAMPLE_RANGE;
    double y = car_pose_.position.y + uni_dist_(gen_)*Y_SAMPLE_RANGE;

    // sample recursively until one in the free space gets returned
    if (!occupancy_grid::is_xy_occupied(map_updated_,x, y)){
        sampled_point.push_back(x);
        sampled_point.push_back(y);
        return sampled_point;
    }
    else{
        return sample();
    }
}

int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    double min_dist2 = 100000.0;
    for (int i=0; i<tree.size(); i++){
        double dist2 = calculate_dist2(tree.at(i).x, sampled_point[0], tree.at(i).y, sampled_point[1]);
        if (dist2<min_dist2){
            nearest_node = i;
            min_dist2 = dist2;
        }
    }
    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering
    Node new_node;
    double dist = sqrt(calculate_dist2(nearest_node.x, sampled_point[0], nearest_node.y, sampled_point[1]));

    new_node.x = nearest_node.x + min(STEER_RANGE, dist)*(sampled_point[0]-nearest_node.x)/dist;
    new_node.y = nearest_node.y + min(STEER_RANGE, dist)*(sampled_point[1]-nearest_node.y)/dist;

    return new_node;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    new_node (Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    using namespace occupancy_grid;
    int x_cell_diff = abs(ceil((nearest_node.x-new_node.x)/map_updated_.info.resolution));
    int y_cell_diff = abs(ceil((nearest_node.y-new_node.y)/map_updated_.info.resolution));

    double dt = 1.0/max(x_cell_diff, y_cell_diff);
    double t = 0.0;
    for (int i=0;i<= max(x_cell_diff, y_cell_diff); i++){
        double x = nearest_node.x + t*(new_node.x-nearest_node.x);
        double y = nearest_node.y + t*(new_node.y-nearest_node.y);
        if (is_xy_occupied(map_updated_, x, y)){
            collision = true;
            break;
        }
        t+=dt;
    }
    return collision;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    return calculate_dist2(goal_x, latest_added_node.x, goal_y, latest_added_node.y) < pow(GOAL_THRESHOLD, 2);
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    std::vector<Node> found_path;
    Node current = latest_added_node;
    while (!tree.at(current.parent).is_root){
        found_path.push_back(current);
        current = tree.at(current.parent);
    }
    found_path.push_back(tree.at(current.parent)); // add start node
    reverse(found_path.begin(), found_path.end());
    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<Node>): the current tree
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node
    double cost = 0;
    // TODO: fill in this method

    return cost;
}

double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<Node>): the current tree
    //   node (Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    return neighborhood;
}

void RRT::visualize_tree(vector<Node>& tree){
    visualization_msgs::Marker goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.id = 0;
    goal.ns = "goal";
    goal.type = visualization_msgs::Marker::POINTS;
    goal.scale.x = goal.scale.y = 0.2;
    goal.scale.z = 0.2;
    goal.action = visualization_msgs::Marker::ADD;
    goal.pose.orientation.w = 1.0;
    goal.color.r = 1.0;
    goal.color.a = 1.0;
    // dots.lifetime = ros::Duration();

    goal.points.push_back(waypoints_.at(curr_goal_ind_));
    tree_viz_pub_.publish(goal);
}

void RRT::track_path(const nav_msgs::Path& path){
    //use pure pursuit to track the path planned by RRT
    int i =0;

    while (i<path.poses.size()-1){
        float x = path.poses.at(i).pose.position.x;
        float y = path.poses.at(i).pose.position.y;
        float x_car = car_pose_.position.x;
        float y_car = car_pose_.position.y;
        if (calculate_dist2(x, x_car, y, y_car) > pow(LOOK_AHEAD_DIST, 2)){
            break;
        }
        i++;
    }
    //calculate setpoint for pure pursuit to track

    tf::Vector3 p1(path.poses.at(i).pose.position.x, path.poses.at(i).pose.position.y, 0.0);
    tf::Vector3 p2(path.poses.at(max(0,i-1)).pose.position.x, path.poses.at(max(0,i-1)).pose.position.y, 0.0);

    pos_sp_ = tf_.inverse() * ((p1 + p2) / 2.0);
    float curvature = 2*abs(pos_sp_.getY())/(LOOK_AHEAD_DIST * LOOK_AHEAD_DIST);

    float steering_cmd =  P_GAIN * curvature;
    if (pos_sp_.getY()<0){steering_cmd *= -1.0;}
    publish_cmd(steering_cmd);

    // visualize pos_sp
    visualization_msgs::Marker pos_sp;
    pos_sp.header.stamp = ros::Time::now();
    pos_sp.header.frame_id = "map";
    pos_sp.id = 1;
    pos_sp.ns = "pos_sp";
    pos_sp.type = visualization_msgs::Marker::SPHERE;
    pos_sp.scale.x = pos_sp.scale.y = 0.2;
    pos_sp.scale.z = 0.2;
    pos_sp.action = visualization_msgs::Marker::ADD;
    pos_sp.pose.orientation.w = 1.0;
    pos_sp.color.b = 1.0;
    pos_sp.color.a = 1.0;
    // dots.lifetime = ros::Duration();

    pos_sp.pose.position.x = (tf_*pos_sp_).x();
    pos_sp.pose.position.y = (tf_*pos_sp_).y();

    pos_sp_viz_pub_.publish(pos_sp);
}

void RRT::publish_cmd(float steering_cmd){
    steering_cmd = min(steering_cmd, 0.41f);
    steering_cmd = max(steering_cmd, -0.41f);

    ackermann_msgs::AckermannDriveStamped ack_msg;
    ack_msg.header.stamp = ros::Time::now();
    ack_msg.drive.speed = SPEED;
    ack_msg.drive.steering_angle = steering_cmd;
    ack_msg.drive.steering_angle_velocity = 1.0;
    drive_pub_.publish(ack_msg);
}