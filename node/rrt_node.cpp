// AUthor: Yuwei Wang

#include "rrt/rrt.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt_node");
    ros::NodeHandle nh;
    RRT rrt(nh);
//    ros::Rate rate(1);
//    while(ros::ok()){
//        ros::spinOnce();
//        rate.sleep();
//    }
    ros::spin();
    return 0;
}
