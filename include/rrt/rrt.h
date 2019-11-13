// Author: Yuwei Wang

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
#include <nav_msgs/Path.h>
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

    ros::Publisher drive_pub_;
    ros::Publisher obstacle_viz_pub_;
    ros::Publisher path_pub_;
    ros::Publisher tree_viz_pub_;
    ros::Publisher pos_sp_viz_pub_;

    tf::TransformListener listener;
    tf::Transform tf_;
    tf::Vector3 pos_sp_;   // setpoint for pure pursuit to track

    nav_msgs::OccupancyGrid map_;
    nav_msgs::OccupancyGrid map_updated_;

    geometry_msgs::Pose car_pose_;
    std::vector<geometry_msgs::Point> waypoints_;
    int curr_goal_ind_;
    bool advance_goal_;

    std::mt19937 gen_;

    std::uniform_real_distribution<> uni_dist_;

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    void init_occupancy_grid();
    void visualize_map();
    void load_waypoints(std::string file_name);

    //RRT
    void get_current_goal();
    void reset_goal();
    void advance_goal();
    int find_closest_waypoint(const std::vector<geometry_msgs::Point>& waypoints, const geometry_msgs::Pose& pose);

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
    void visualize_tree(std::vector<Node>& tree);
    void track_path(const nav_msgs::Path& path);
    void publish_cmd(float steering_cmd);
};