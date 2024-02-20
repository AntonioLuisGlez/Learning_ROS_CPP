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

namespace catec {

// Class definition for MarkersNode
class MarkersNode
{
  public:
    // Constructor and Destructor
    MarkersNode();
    ~MarkersNode();

    // Initialize the node
    void init();

    // Perform a step in the node's operation
    void step();

  private:
    ros::NodeHandle            nh_;         // ROS NodeHandle for the MarkersNode
    ros::Publisher             marker_pub_; // ROS Publisher for markers
    visualization_msgs::Marker marker_;     // Marker object for visualization
};

// Class definition for PointCloudGenerator
class PointCloudGenerator
{
  public:
    // Constructor for PointCloudGenerator
    PointCloudGenerator();

    // Method to perform a step
    void step();

  private:
    ros::NodeHandle nh_;              // ROS NodeHandle for the PointCloudGenerator
    ros::Publisher  point_cloud_pub_; // ROS Publisher for point clouds

    // Generate the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr generatePointCloud();

    // Publish the generated point cloud
    void publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  private:
    // Pointer to an object of type ROSMessageGenerator may be intended here
    // std::unique_ptr<ROSMessageGenerator> _msg_generator;
};

class CircularPathGenerator
{
  public:
    CircularPathGenerator();

    // Método para generar y publicar la trayectoria circular
    void step();

  private:
    ros::NodeHandle               nh_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    double                        radius_;
    double                        angular_velocity_;
    std::string                   frame_id_;
    std::string                   child_frame_id_;

    // Método privado para publicar la transformación entre los sistemas de referencia odom y base_link
    void publishTransform(double x, double y);
};

} // namespace catec
