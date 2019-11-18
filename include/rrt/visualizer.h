//
// Created by yuwei on 11/14/19.
//

#ifndef SRC_VISUALIZATION_H
#define SRC_VISUALIZATION_H

#endif //SRC_VISUALIZATION_H

#pragma once

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

using namespace std;

static int num_visuals = 0;

class PointsVisualizer {
protected:
    ros::Publisher& pub;
    visualization_msgs::Marker dots;
    string ns;
    string frame_id;

public:
    PointsVisualizer(ros::Publisher& pub, string ns, string frame_id, std_msgs::ColorRGBA color, float scale):
                        pub(pub), ns(ns),frame_id(frame_id){
        dots.header.frame_id = frame_id;
        dots.ns = ns;
        dots.action = visualization_msgs::Marker::ADD;
        dots.pose.orientation.w = 1.0;
        dots.id = num_visuals;
        dots.type = visualization_msgs::Marker::POINTS;
        dots.scale.x = dots.scale.y = dots.scale.z = scale;
        dots.color = color;
       // dots.color.r = 1.0; dots.color.a = 1.0;
        ++num_visuals;
    };

    void add_point(geometry_msgs::Point p) {
        dots.points.push_back(p);
    };
    void publish_points(){
        pub.publish(dots);
        ROS_INFO("published dots");
        dots.points.clear();
    };
};

class MarkerVisualizer {
protected:
    ros::Publisher& pub;
    visualization_msgs::Marker dot;
    string ns;
    string frame_id;

public:
    MarkerVisualizer(ros::Publisher& pub, string ns, string frame_id, std_msgs::ColorRGBA color, float scale, int shape):
            pub(pub), ns(ns),frame_id(frame_id){
        dot.header.frame_id = frame_id;
        dot.ns = ns;
        dot.action = visualization_msgs::Marker::ADD;
        dot.id = num_visuals;
        dot.type = shape;
        dot.scale.x = dot.scale.y = dot.scale.z = scale;
        dot.color = color;
        // dots.color.r = 1.0; dots.color.a = 1.0;
        ++num_visuals;
    };

    void set_pose(geometry_msgs::Pose pose){
        dot.pose.orientation = pose.orientation;
        dot.pose.position = pose.position;
    }
    void publish_marker(){
        pub.publish(dot);
    };
};


class LineListVisualizer{
protected:
    ros::Publisher& pub;
    visualization_msgs::Marker line_list;
    string ns;
    string frame_id;

public:
    LineListVisualizer(ros::Publisher& pub, string ns, string frame_id);
    void add_line(geometry_msgs::Point p1, geometry_msgs::Point p2);
    void publish_line_list();
};


//LineListVisualizer::LineListVisualizer(ros::Publisher& pub, string ns, string frame_id) : pub(pub), ns(ns),
//                                                                                          frame_id(frame_id) {
//    line_list.header.frame_id = frame_id;
//    line_list.ns = ns;
//    line_list.action = visualization_msgs::Marker::ADD;
//    line_list.pose.orientation.w = 1.0;
//    line_list.id = num_visuals;
//    line_list.type = visualization_msgs::Marker::LINE_LIST;
//    line_list.scale.x = 0.01;
//    line_list.color.r =0.0; line_list.color.b = 1.0; line_list.color.g = 0; line_list.color.a = 1.0;
//    num_visuals++;
//}
//
//void LineListVisualizer::add_line(geometry_msgs::Point p1, geometry_msgs::Point p2) {
//    line_list.points.push_back(p1);
//    line_list.points.push_back(p2);
//}
//
//void LineListVisualizer::publish_line_list() {
//    line_list.header.stamp = ros::Time::now();
//    pub.publish(line_list);
//    ROS_INFO("published dots");
//    line_list.points.clear();
//    line_list.colors.clear();
//}
