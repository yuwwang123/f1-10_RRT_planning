// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

// Struct defining the Node object in the RRT tree.
// More fields could be added to this struct if more info needed.
// You can choose to use this or not
typedef struct Node {
    double x, y;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} Node;


class RRT {
public:
    RRT(ros::NodeHandle &nh);
    virtual ~RRT();
private:
    ros::NodeHandle nh_;

    // ros pub/sub


    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher obstacle_viz_pub_;
    ros::Publisher path_pub_;
    ros::Publisher tree_viz_pub_;
    // tf stuff
    tf::TransformListener listener;

    nav_msgs::OccupancyGrid map_;
    nav_msgs::OccupancyGrid map_updated_;

    geometry_msgs::Pose car_pose_;
    // random generator, use this

    std::mt19937 gen_;
   // std::uniform_real_distribution<> x_distribution;
   // std::uniform_real_distribution<> y_distribution;
   std::uniform_real_distribution<> uni_dist_;

    // callbacks
    // where rrt actually happens
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    // RRT methods
    void init_occupancy_grid();
    void visualize_map();
    void visualize_tree(std::vector<Node>& tree);
    std::vector<double> sample();
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
    Node steer(Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(Node &nearest_node, Node &new_node);
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);
    std::vector<Node> find_path(std::vector<Node> &tree, Node &latest_added_node);
    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);

};

