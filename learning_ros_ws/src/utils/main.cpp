#include <ros/ros.h>

#include <memory>

#include "learning_ros_cpp/markers_pcl_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_and_pointcloud_node");

    // Crea un puntero único para MarkersNode y lo inicializa
    std::unique_ptr<catec::MarkersNode> markers_node = std::make_unique<catec::MarkersNode>();
    markers_node->init();

    // Crea un puntero único para PointCloudGenerator y lo inicializa
    std::unique_ptr<catec::PointCloudGenerator> point_cloud_generator = std::make_unique<catec::PointCloudGenerator>();

    // Crea un puntero único para CircularPathGenerator y lo inicializa
    std::unique_ptr<catec::CircularPathGenerator> circular_path_generator
            = std::make_unique<catec::CircularPathGenerator>();

    // Establece la frecuencia de publicación a 10 Hz
    ros::Rate rate(10);

    // Bucle principal
    while (ros::ok()) {
        markers_node->step();            // Realiza un paso en MarkersNode
        point_cloud_generator->step();   // Realiza un paso en PointCloudGenerator
        circular_path_generator->step(); // Genera y publica la trayectoria circular

        // Espera hasta la próxima iteración basado en la frecuencia
        rate.sleep();
    }

    return 0;
}
