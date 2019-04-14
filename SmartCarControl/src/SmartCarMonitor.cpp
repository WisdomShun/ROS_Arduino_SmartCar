//
// Created by shun on 19-4-13.
//

#include "SmartCarMonitor.h"
#include <ros/ros.h>

void Listener::callback(const geometry_msgs::Twist::ConstPtr &msg) {
    ROS_DEBUG("linear : [x = %f] \tangular : [z = %f]", msg->linear.x, msg->angular.z);
    if (abs(msg->angular.z - 2.0) < 0.000001) {
        ROS_INFO("turning left");
    } else if (abs(msg->angular.z + 2.0) < 0.000001) {
        ROS_INFO("turning right");
    } else if (abs(msg->linear.x - 2.0) < 0.000001) {
        ROS_INFO("forwarding");
    } else if (abs(msg->linear.x + 2.0) < 0.000001) {
        ROS_INFO("backwarding");
    } else {
        ROS_ERROR("ERROR Twist Information!");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Listen");
    ros::NodeHandle nh;

    Listener listener;
    ros::Subscriber sub = nh.subscribe("smartcar/cmd_vel", 1000, &Listener::callback, &listener);
    ros::spin();

    return 0;
}