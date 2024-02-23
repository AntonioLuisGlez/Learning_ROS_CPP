#include "learning_ros_cpp/subscriber_node.h"

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <iostream>

RosSubscribers::RosSubscribers(const std::shared_ptr<ros::NodeHandle>& nh) : _nh(std::make_shared<ros::NodeHandle>(*nh))
{
    std::string odometry_topic, orientation_topic;
    _nh->param<std::string>("odometry_topic", odometry_topic, "/odometry");
    _nh->param<std::string>("orientation_topic", orientation_topic, "/orientation");

    _odometry_sub    = _nh->subscribe(odometry_topic, 1, &RosSubscribers::odom_callback, this);
    _orientation_sub = _nh->subscribe(orientation_topic, 1, &RosSubscribers::quaternion_callback, this);
}

void RosSubscribers::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::cout << "Received Odometry message: [" << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y
              << ", " << msg->pose.pose.position.z << "]" << std::endl;
}

void RosSubscribers::quaternion_callback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    std::cout << "Received Quaternion message: [" << msg->x << ", " << msg->y << ", " << msg->z << ", " << msg->w << "]"
              << std::endl;
}