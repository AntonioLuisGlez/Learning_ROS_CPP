#include "learning_ros_cpp/pkg_node.h"

#include "learning_ros_cpp/ros_publishers.h"

PkgNode::PkgNode() : nh_(std::make_shared<ros::NodeHandle>("~")), publisher_(std::make_unique<RosPublishers>(nh_)) {}

void PkgNode::run()
{
    const auto namespace_info = nh_->getNamespace();
    std::cout << "namespace: " << namespace_info << std::endl;

    // Configura el bucle principal
    ros::Rate rate(10);
    while (ros::ok()) {
        /*
        // Llama a los métodos para publicar desde la clase RosPublishers
        publisher_->publishMarker();
        publisher_->publishPointCloud();
        publisher_->publishCircularPath();
        */

        publisher_->publishMarker();
        publisher_->publishPointCloud();
        publisher_->publishCircularPath();

        rate.sleep();
    }
}
