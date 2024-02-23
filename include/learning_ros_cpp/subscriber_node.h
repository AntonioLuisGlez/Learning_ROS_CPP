#pragma once
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class RosSubscribers
{
  public:
    RosSubscribers(const std::shared_ptr<ros::NodeHandle>& nh);

    nav_msgs::Odometry getOdometry();

  private:
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void quaternion_callback(const geometry_msgs::Quaternion::ConstPtr& msg);

  private:
    std::shared_ptr<ros::NodeHandle> _nh;

    ros::Subscriber _odometry_sub;
    ros::Subscriber _orientation_sub;

    nav_msgs::Odometry _received_odometry;
};
