// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"
#include <rrt/occupancy_grid.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

using namespace std;

const float MARGIN = 0.2;
const int MAX_ITER = 200;
const double X_SAMPLE_RANGE = 3.0;
const double Y_SAMPLE_RANGE = 3.0;
const double GOAL_THRESHOLD = 0.2;
const double STEER_RANGE = 0.5;
const float SCAN_RANGE = sqrt(pow(X_SAMPLE_RANGE,2.0)+ pow(Y_SAMPLE_RANGE, 2.0));

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen_((std::random_device())()) {

    std::string pose_topic, scan_topic;

    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);

    // ROS publishers

    // ROS subscribers

    odom_sub_ = nh_.subscribe("/odom", 10, &RRT::odom_callback, this);
    scan_sub_ = nh_.subscribe("/scan", 10, &RRT::scan_callback, this);

    obstacle_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("obstacles_inflated", 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>("path_found", 10);
    tree_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("rrt_tree", 10);

    std::uniform_real_distribution<double> distribution(-1.0,1.0);
    uni_dist_ = distribution;

    init_occupancy_grid();
    visualize_map();

    ROS_INFO("Created new RRT Object.");
}

void RRT::init_occupancy_grid(){

    boost::shared_ptr<nav_msgs::OccupancyGrid const > map_ptr;
    map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(5.0));
    if (map_ptr == NULL){ROS_INFO("No map received");}
    else{
        map_ = *map_ptr;
        map_updated_ = map_;
        cout<<" map received. "<<endl;
    }
    float margin;
    nh_.getParam("obstacle_margin", margin);
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
    //cout<<" dots size:  "<< dots.<<endl;
    obstacle_viz_pub_.publish(dots);
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
    tf::Transform tf;
    tf.setOrigin(origin);
    tf.setRotation(q_tf);

    map_updated_ = map_; // might be expensive to copy

    float angle_min = msg->angle_min;
    float angle_increment = msg->angle_increment;

    const vector<float>& ranges = msg->ranges;

    for (int i = 0; i < ranges.size(); ++i) {
        float range = ranges.at(i);
        if (range > SCAN_RANGE) {
            continue;
        }
        if (!isnan(range) && !isinf(range)) {
            float angle = angle_min + angle_increment * i;
            tf::Vector3 pos_in_car = tf::Vector3(range*cos(angle), range*sin(angle), 0.0);
            tf::Vector3 pos_in_map = tf * pos_in_car;
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
    // Returns:
    //
    //Get start
    car_pose_ = odom_msg->pose.pose;
    Node start_node;
    start_node.x = car_pose_.position.x;
    start_node.y = car_pose_.position.y;
    start_node.parent = 0;
    start_node.is_root = true;
    //Get Goal
    //simple test: stationary goal
    geometry_msgs::Point curr_goal;

    curr_goal.x = 9.693; curr_goal.y = 2.2; curr_goal.z = 0;
    // tree as std::vector
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
            if (is_goal(new_node, curr_goal.x, curr_goal.y)){
                ROS_INFO("Path found");
                break;
            }
        }
        if (i==MAX_ITER-1){
            no_path = true;
            ROS_INFO("Couldn't find a path");
        }
    };

    // path found as Path message
    vector<Node> path_found = find_path(tree, latest_node);
    nav_msgs::Path path;
    path.header.frame_id = "map";
    for (int i=0; i<path_found.size(); i++){
        geometry_msgs::PoseStamped p;
        p.header.frame_id = "map";
        p.pose.position.x = path_found.at(i).x;
        p.pose.position.y = path_found.at(i).y;
        p.pose.orientation.w = 1.0;
        path.poses.push_back(p);
    }
    if (!no_path){
        path_pub_.publish(path);
    }
    visualize_tree(tree);
}

void RRT::visualize_tree(vector<Node>& tree){
    visualization_msgs::Marker goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.id = 0;
    goal.ns = "goal";
    goal.type = visualization_msgs::Marker::POINTS;
    goal.scale.x = goal.scale.y = 0.1;
    goal.scale.z = 0.1;
    goal.action = visualization_msgs::Marker::ADD;
    goal.pose.orientation.w = 1.0;
    goal.color.r = 1.0;
    goal.color.a = 1.0;
   // dots.lifetime = ros::Duration();

    geometry_msgs::Point curr_goal;
    curr_goal.x = 9.693; curr_goal.y = 2.2; curr_goal.z = 0;
    goal.points.push_back(curr_goal);
    tree_viz_pub_.publish(goal);
}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)
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
    double min_dist = 10000.0;
    for (int i=0; i<tree.size(); i++){
        double dist = pow((tree.at(i).x-sampled_point[0]),2.0) + pow((tree.at(i).y-sampled_point[1]),2.0);
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
    double dist = sqrt(pow(nearest_node.x-sampled_point[0],2.0)+pow(nearest_node.y-sampled_point[1],2.0));

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
    // Args:
    //   latest_added_node (Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    return pow(goal_x-latest_added_node.x,2.0) + pow(goal_y-latest_added_node.y,2.0) < GOAL_THRESHOLD*GOAL_THRESHOLD;
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
    found_path.push_back(tree.at(current.parent));
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