//Author: Yuwei Wang

#include "rrt/rrt.h"
#include <rrt/occupancy_grid.h>
#include <rrt/CSVReader.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <rrt/spline.h>
#include <tf/transform_listener.h>

using namespace std;

//const float MARGIN = 0.18;
//const float DETECTED_OBS_MARGIN = 0.2;
const int MAX_ITER = 1200;
const int MIN_ITER = 1000;
const double X_SAMPLE_RANGE = 3;
const double Y_SAMPLE_RANGE = 3;
const double STD = 1.5;   // standard deviation for normal distribution
const double GOAL_THRESHOLD = 0.15;
const double STEER_RANGE = 0.3;
//const float SCAN_RANGE = 3.0;
const float GOAL_AHEAD_DIST = 3.5;
//const float LOOK_AHEAD_DIST = 0.4;
//const float P_GAIN = 0.3;
//const float SPEED = 2.7;
const float NEAR_RANGE = 1.0;

const string file_name = "/home/yuwei/rcws/logs/yuwei_wp.csv";

//const string pose_topic = "pf/pose/odom";
const string pose_topic = "odom";

const string scan_topic = "scan";
//const string drive_topic = "/vesc/high_level/ackermann_cmd_mux/input/nav_1";
const string drive_topic = "/nav";



RRT::~RRT() {
    ROS_INFO("RRT shutting down");
}

RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen_((std::random_device())()) {

    // ROS publishers & subscribers
    odom_sub_ = nh_.subscribe(pose_topic, 10, &RRT::odom_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);

    path_pub_ = nh_.advertise<visualization_msgs::Marker>("path_found", 1);
    map_update_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map_updated", 5);

    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    obstacle_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("obstacles_inflated", 1);


    tree_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("rrt_tree", 1);

    pos_sp_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("pos_sp_", 1);
    goal_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("goal", 1);
    tree_nodes_pub_ = nh_.advertise<visualization_msgs::Marker>("tree_nodes", 1);
    tree_branches_pub_ = nh_.advertise<visualization_msgs::Marker>("tree_branches", 1);

    std::uniform_real_distribution<double> unit_dist(-1.0,1.0);
    uni_dist_ = unit_dist;


    std_msgs::ColorRGBA red; red.r =1.0; red.a=1.0;
    std_msgs::ColorRGBA green; green.g =1.0; green.a=1.0;
    std_msgs::ColorRGBA blue; blue.b =1.0; blue.a=1.0;

    pos_sp_viz = new MarkerVisualizer(pos_sp_viz_pub_, "pos_sp", "laser", red, 0.2, visualization_msgs::Marker::SPHERE);
    goal_viz = new MarkerVisualizer(goal_viz_pub_, "goal", "map", green, 0.3, visualization_msgs::Marker::SPHERE);

    tree_nodes.header.frame_id = tree_branch.header.frame_id = "map";
    tree_nodes.ns = "nodes"; tree_branch.ns = "branch";
    tree_nodes.action = tree_branch.action = visualization_msgs::Marker::ADD;
    tree_nodes.pose.orientation.w = tree_branch.pose.orientation.w = 1.0;
    tree_nodes.id = 5; tree_branch.id = 6;
    tree_nodes.type = visualization_msgs::Marker::POINTS;
    tree_branch.type = visualization_msgs::Marker::LINE_LIST;
    tree_nodes.scale.x = tree_nodes.scale.y = tree_nodes.scale.z = 0.05;
    tree_branch.scale.x = 0.01;
    tree_nodes.color = red; tree_branch.color = blue;

    init_occupancy_grid();
    visualize_map();
    load_waypoints(file_name);
    reset_goal();
    ROS_INFO("Created new RRT Object.");
    last_time_ = ros::Time::now().toSec();


    nh_.param<float>("/rrt_node/SPEED", SPEED, 5.0);
    nh_.param<float>("/rrt_node/MARGIN", MARGIN, 0.25);
    nh_.param<float>("/rrt_node/DETECTED_OBS_MARGIN",  DETECTED_OBS_MARGIN, 0.2);
    nh_.param<float>("/rrt_node/P_GAIN", P_GAIN, 0.3);
    nh_.param<float>("/rrt_node/SCAN_RANGE", SCAN_RANGE, 3.5);
    nh_.param<float>("/rrt_node/LOOK_AHEAD_DIST", LOOK_AHEAD_DIST, 0.5);
    nh_.param<float>("/rrt_node/MAX_DECELARATION", MAX_DECELARATION, 0.7*SPEED);

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
    occupancy_grid::inflate_map(map_, MARGIN);
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

    using namespace occupancy_grid;

    tf::StampedTransform tf_stamped;
    listener_.lookupTransform("/map", "/laser", ros::Time(0), tf_stamped);
    tf_.setOrigin(tf_stamped.getOrigin());
    tf_.setRotation(tf_stamped.getRotation());

    //only reset map when the car has made enough progress
//    if(ros::Time::now().toSec() - last_time_ > 5.0){
//        map_updated_ = map_; // might be expensive to copy
//        last_time_ = ros::Time::now().toSec();
//    }
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
            tf::Vector3 pos_in_car(range*cos(angle), range*sin(angle), 0.0);
            tf::Vector3 pos_in_map = tf_ * pos_in_car;
            if (!is_xy_occupied(map_, pos_in_map.x(), pos_in_map.y())){
                inflate_cell(map_updated_, xy2ind(map_updated_, pos_in_map.x(), pos_in_map.y()), DETECTED_OBS_MARGIN, 100);
            }
        }
    }
    // free the cells in which the car occupies (dynamic layer)
    inflate_cell(map_updated_, xy2ind(map_updated_, car_pose_.position.x, car_pose_.position.y), 0.25, 0);
    map_update_pub_.publish(map_updated_);
}

bool comp_cost(Node& n1, Node& n2){
    return n1.cost < n2.cost;
}

void RRT::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    car_pose_ = odom_msg->pose.pose;
    Node start_node;
    start_node.x = car_pose_.position.x;
    start_node.y = car_pose_.position.y;
    start_node.parent = 0;
    start_node.cost = 0.0;
    start_node.is_root = true;
    //Get Goal
    get_current_goal();

    std::vector<Node> tree;
    std::vector<Node> nodes_near_goal;

    tree.clear();
    tree.push_back(start_node);
    /***  RRT* main loop ***/
    for (int iter=0; iter<MAX_ITER; iter++){
        vector<double> sampled_point = sample();
        int nearest_ind = nearest(tree, sampled_point);
        Node new_node = steer(tree.at(nearest_ind), sampled_point);
        if (!check_collision(tree.at(nearest_ind), new_node)){
            vector<int> nodes_near = near(tree, new_node);
            tree.push_back(new_node);

            /** connect new_node to the node in the neighborhood with the minimum cost **/
            int min_cost_node_ind = nearest_ind;
            float min_cost = tree.at(nearest_ind).cost + line_cost(tree.at(nearest_ind), new_node);
            for (int i=0; i<nodes_near.size(); i++){
                if(!check_collision(tree.at(nodes_near.at(i)), new_node)){
                    float cost = tree.at(nodes_near.at(i)).cost + line_cost(tree.at(nodes_near.at(i)), new_node);
                    if(cost < min_cost) {
                       min_cost_node_ind = nodes_near.at(i);
                       min_cost = cost;
                   }
                }
            }

            tree.back().is_root = false;
            tree.back().cost = min_cost;
            // add edge
            tree.back().parent = min_cost_node_ind;
            tree.at(min_cost_node_ind).children.push_back(tree.size()-1);

            /** Rewiring **/
            int rewire_count = 0;
            for (int j=0; j<int(nodes_near.size()); j++) {
                float new_cost = tree.back().cost + line_cost(new_node, tree.at(nodes_near.at(j)));
                if (new_cost < tree.at(nodes_near.at(j)).cost) {
                    if (!check_collision(tree.at(nodes_near.at(j)), new_node)) {
                        // rewire: update parent, cost and costs of all children;
                        float cost_change = new_cost - tree.at(nodes_near.at(j)).cost;
                        tree.at(nodes_near.at(j)).cost = new_cost;
                        // assign new_node to be its new parent
                        int old_parent = tree.at(nodes_near.at(j)).parent;
                        tree.at(nodes_near.at(j)).parent = tree.size() - 1;
                        tree.back().children.push_back(nodes_near.at(j));
                        // remove it from its old parent's children list
                        vector<int>::iterator start = tree.at(old_parent).children.begin();
                        vector<int>::iterator end = tree.at(old_parent).children.end();
                        tree.at(old_parent).children.erase(remove(start, end, nodes_near.at(j)), end);
                        // update_children_cost(tree, nodes_near.at(j), cost_change); // optional(expensive)
                    }
                }
                rewire_count ++;
            }
            //cout<<"rewire: "<<rewire_count<<endl;
            if (is_goal(tree.back(), waypoints_.at(curr_goal_ind_).x, waypoints_.at(curr_goal_ind_).y)){
                nodes_near_goal.push_back(tree.back());
            }
        }
        /** check if goal reached and recover path with the minimum cost**/
        if(iter>MIN_ITER && !nodes_near_goal.empty()){
            Node best = *min_element(nodes_near_goal.begin(), nodes_near_goal.end(), comp_cost);
            vector<Node> path_found = find_path(tree, nodes_near_goal.back());

            visualization_msgs::Marker path_dots;
            path_dots.header.frame_id = "map";
            path_dots.id = 20;
            path_dots.ns = "path";
            path_dots.type = visualization_msgs::Marker::POINTS;
            path_dots.scale.x = path_dots.scale.y = path_dots.scale.z = 0.08;
            path_dots.action = visualization_msgs::Marker::ADD;
            path_dots.pose.orientation.w = 1.0;
            path_dots.color.g = 0.0;
            path_dots.color.r = 1.0;
            path_dots.color.a = 1.0;

            for (int i=0; i<path_found.size(); i++){
                geometry_msgs::Point p;
                p.x = path_found.at(i).x;
                p.y = path_found.at(i).y;
                path_dots.points.push_back(p);
            }
            double RRT_INTERVAL = 0.2;
            vector<geometry_msgs::Point> path_processed;
            for (int i=0; i< path_dots.points.size()-1; i++){
                path_processed.push_back(path_dots.points[i]);
                double dist = sqrt(pow(path_dots.points[i+1].x-path_dots.points[i].x, 2)
                                   +pow(path_dots.points[i+1].y-path_dots.points[i].y, 2));
                if (dist < RRT_INTERVAL) continue;
                int num = static_cast<int>(ceil(dist/RRT_INTERVAL));
                for(int j=1; j< num; j++){
                    geometry_msgs::Point p;
                    p.x = path_dots.points[i].x + j*((path_dots.points[i+1].x - path_dots.points[i].x)/num);
                    p.y = path_dots.points[i].y + j*((path_dots.points[i+1].y - path_dots.points[i].y)/num);
                    path_processed.push_back(p);
                }
            }

            path_dots.points = path_processed;
            path_pub_.publish(path_dots);
//            track_path(path);
            visualize_tree(tree);
            //ROS_INFO("path found");
            break;
        }
    }

    if (nodes_near_goal.empty()){
        ROS_INFO("Couldn't find a path");
    }
}

double calculate_dist2(double x1, double x2, double y1, double y2){
    // considering that doing sqrt is expensive
    return pow(x1-x2, 2) + pow(y1-y2,2);
}

void RRT::get_current_goal(){
    float dist_to_goal2 = calculate_dist2(waypoints_.at(curr_goal_ind_).x, car_pose_.position.x,
                              waypoints_.at(curr_goal_ind_).y,  car_pose_.position.y);
    // goal out of range, reset goal
    if (dist_to_goal2 > pow(GOAL_AHEAD_DIST, 2)){
        reset_goal();
    }
    if (occupancy_grid::is_xy_occupied(map_updated_, waypoints_.at(curr_goal_ind_).x, waypoints_.at(curr_goal_ind_).y)){
        advance_goal();
    }
    // enough progress made, advance goal
    else if(dist_to_goal2 < pow(GOAL_AHEAD_DIST*0.75, 2)){
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
        if (closest_dist2 > pow(GOAL_AHEAD_DIST, 2)){
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

    while(curr_dist2 < pow(GOAL_AHEAD_DIST*0.9, 2)
      || is_xy_occupied(map_updated_, waypoints_.at(curr_ind).x, waypoints_.at(curr_ind).y)){
        curr_dist2 = calculate_dist2(waypoints_.at(curr_ind).x, pose_x, waypoints_.at(curr_ind).y, pose_y);
        curr_ind++;
        if (curr_ind >= waypoints_.size()) {curr_ind = 0;}
        if (curr_dist2 > pow(GOAL_AHEAD_DIST, 2) && is_xy_occupied(map_updated_, waypoints_.at(curr_ind).x, waypoints_.at(curr_ind).y)){
            break;
        }
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

   // double x = car_pose_.position.x + uni_dist_(gen_)*X_SAMPLE_RANGE;
   // double y = car_pose_.position.y + uni_dist_(gen_)*Y_SAMPLE_RANGE;
    std::normal_distribution<double> norm_dist_x(0.6*waypoints_.at(curr_goal_ind_).x+0.4*car_pose_.position.x, STD);
    std::normal_distribution<double> norm_dist_y(0.6*waypoints_.at(curr_goal_ind_).y+0.4*car_pose_.position.y, STD);
    double x = norm_dist_x(gen_);
    double y = norm_dist_y(gen_);

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
    double min_dist = 100000.0;
    for (int i=0; i<int(tree.size()); i++){
        double dist = calculate_dist2(tree.at(i).x, sampled_point[0], tree.at(i).y, sampled_point[1]);
        if (dist<min_dist){
            nearest_node = i;
            min_dist = dist;
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
    // enough
    return calculate_dist2(goal_x, latest_added_node.x, goal_y, latest_added_node.y) < pow(GOAL_THRESHOLD, 2);
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node& node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    std::vector<Node> found_path;
    Node current = node;
    while (!current.is_root){
        found_path.push_back(current);
        current = tree.at(current.parent);
    }
    found_path.push_back(current); // add start node
    reverse(found_path.begin(), found_path.end());
    return found_path;
}

// RRT* methods
double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes

    return sqrt(calculate_dist2(n1.x, n2.x, n1.y, n2.y));
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node& node) {
    // This method returns the set of Nodes in the neighborhood of a node.
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood
    std::vector<int> neighborhood;
    neighborhood.clear();
    for (int i=0; i<tree.size(); i++){
        if (line_cost(tree.at(i), node) < NEAR_RANGE){
            neighborhood.push_back(i);
        }
    }
    return neighborhood;
}

void RRT::visualize_tree(vector<Node>& tree){
    // plot goal first
    geometry_msgs::Pose goal_pose;
    goal_pose.orientation.w = 1.0;
    goal_pose.position.x = waypoints_.at(curr_goal_ind_).x;
    goal_pose.position.y = waypoints_.at(curr_goal_ind_).y;
    goal_viz->set_pose(goal_pose);
    goal_viz->publish_marker();
    // plot tree
    tree_nodes.points.clear();
    tree_branch.points.clear();

    for (int i=0; i<tree.size(); i++){
        geometry_msgs::Point p;
        p.x = tree.at(i).x; p.y = tree.at(i).y;
        tree_nodes.points.push_back(p);
        for (int j=0; j<tree.at(i).children.size(); j++){
            tree_branch.points.push_back(p);
            geometry_msgs::Point p_child;
            p_child.x = tree.at(tree.at(i).children.at(j)).x;
            p_child.y = tree.at(tree.at(i).children.at(j)).y;
            tree_branch.points.push_back(p_child);
        }
    }
    tree_branches_pub_.publish(tree_branch);
    tree_nodes_pub_.publish(tree_nodes);
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

    // publish drive cmds
    float steering_cmd =  P_GAIN * curvature;
    if (pos_sp_.getY()<0){steering_cmd *= -1.0;}
    publish_cmd(steering_cmd);

    // visualize setpoint for tracking
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = pos_sp_.x(); pose.position.y = pos_sp_.y();
    pos_sp_viz->set_pose(pose);
    pos_sp_viz->publish_marker();
}

void RRT::publish_cmd(float steering_cmd){
    steering_cmd = min(steering_cmd, 0.41f);
    steering_cmd = max(steering_cmd, -0.41f);
    double speed = SPEED;
    if(abs(steering_cmd)>0.28){
        speed -= (steering_cmd-0.28)/(0.41-0.28)*MAX_DECELARATION;
    }
    cout<<"speed: "<<speed<<endl;
    ackermann_msgs::AckermannDriveStamped ack_msg;
    ack_msg.header.stamp = ros::Time::now();
    ack_msg.drive.speed = speed;
    ack_msg.drive.steering_angle = steering_cmd;
    ack_msg.drive.steering_angle_velocity = 1.0;
    drive_pub_.publish(ack_msg);
}

void RRT::update_children_cost(vector<Node>& tree, int root_node_ind, float cost_change){
    if (tree.at(root_node_ind).children.empty()){
        return;
    }
    else{
        for (int i=1; i<tree.at(root_node_ind).children.size(); i++){
            tree.at(tree.at(root_node_ind).children.at(i)).cost += cost_change;
            update_children_cost(tree, tree.at(root_node_ind).children.at(i), cost_change);
        }
    }
}

