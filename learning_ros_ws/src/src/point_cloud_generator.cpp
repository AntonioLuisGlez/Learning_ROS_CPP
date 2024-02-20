#include "learning_ros_cpp/point_cloud_generator.h"

namespace catec {

PointCloudGenerator::PointCloudGenerator() : nh_("~")
{
    // Inicializa el publisher para publicar la nube de puntos
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_topic", 1);
}

void PointCloudGenerator::step()
{
    // Genera la nube de puntos
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloud();
    // Publica la nube de puntos
    publishPointCloud(cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudGenerator::generatePointCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Agrega algunos puntos a la nube de puntos
    for (float x = -1.0; x <= 1.0; x += 0.1) {
        for (float y = -1.0; y <= 1.0; y += 0.1) {
            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = 0.0;
            cloud->points.push_back(point);
        }
    }

    cloud->width  = cloud->points.size();
    cloud->height = 1;

    return cloud;
}

void PointCloudGenerator::publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "sensor2_frame";
    cloud_msg.header.stamp    = ros::Time::now();
    point_cloud_pub_.publish(cloud_msg);
}

} // namespace catec