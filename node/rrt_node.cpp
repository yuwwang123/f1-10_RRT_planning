// AUthor: Yuwei Wang

#include "rrt/rrt.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt");
    ros::NodeHandle nh;
    RRT rrt(nh);
    ros::spin();
    return 0;
}
