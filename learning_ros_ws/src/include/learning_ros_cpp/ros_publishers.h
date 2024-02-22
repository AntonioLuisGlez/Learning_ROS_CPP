// ros_publishers.h
#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

class RosPublishers
{
  public:
    RosPublishers(std::shared_ptr<ros::NodeHandle> nh);
    void publishMarker();
    void publishPointCloud();
    void publishCircularPath();

  private:
    ros::NodeHandle nh_;
    ros::Publisher  marker_pub_;
    ros::Publisher  point_cloud_pub_;

  private:
    tf2_ros::TransformBroadcaster       tf_broadcaster_;
    double                              radius_;
    double                              angular_velocity_;
    std::string                         frame_id_;
    std::string                         child_frame_id_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_; // Miembro para almacenar cloud
};
