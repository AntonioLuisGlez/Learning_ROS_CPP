#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "learning_ros_cpp/ros_subscribers.h"

RosSubscribers::RosSubscribers(const std::shared_ptr<ros::NodeHandle>& nh) : _nh(std::make_shared<ros::NodeHandle>(*nh))
{
    std::string odometry_topic, orientation_topic;
    _nh->param<std::string>("odometry_topic", odometry_topic, "/odometry");
    _nh->param<std::string>("orientation_topic", orientation_topic, "/orientation");

    _odometry_sub    = _nh->subscribe(odometry_topic, 1, &RosSubscribers::odomCb, this);
    _orientation_sub = _nh->subscribe(orientation_topic, 1, &RosSubscribers::quaternionCb, this);
}