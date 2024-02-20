#ifndef POINT_CLOUD_GENERATOR_H
#define POINT_CLOUD_GENERATOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace catec {
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

#endif // POINT_CLOUD_GENERATOR_H
}