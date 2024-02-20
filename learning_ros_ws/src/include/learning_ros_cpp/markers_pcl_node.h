#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

namespace catec {

class MarkersNode
{
  public:
    MarkersNode();
    ~MarkersNode();

    void init();

    void step();

  private:
    ros::NodeHandle            nh_;
    ros::Publisher             marker_pub_;
    visualization_msgs::Marker marker_;
};

class PointCloudGenerator
{
  public:
    PointCloudGenerator();

    void step();

  private:
    ros::NodeHandle nh_;
    ros::Publisher  point_cloud_pub_;

    // Función para generar la nube de puntos
    pcl::PointCloud<pcl::PointXYZ>::Ptr generatePointCloud();

    // Función para publicar la nube de puntos
    void publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  private:
    // std::unique_ptr<ROSMessageGenerator> _msg_generator;
};

} // namespace catec
