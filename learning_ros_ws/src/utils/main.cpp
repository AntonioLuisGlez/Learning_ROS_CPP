#include <ros/ros.h>

#include <memory>

#include "learning_ros_cpp/markers_pcl_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_and_pointcloud_node");

    std::unique_ptr<catec::MarkersNode> markers_node = std::make_unique<catec::MarkersNode>();

    markers_node->init();

    std::unique_ptr<catec::PointCloudGenerator> point_cloud_generator = std::make_unique<catec::PointCloudGenerator>();

    ros::Rate rate(10); // Frecuencia de publicación en Hz
    while (ros::ok()) {
        markers_node->step();
        point_cloud_generator->step();

        // Espera hasta la próxima iteración
        rate.sleep();
    }

    // ros::spin();

    return 0;
}
