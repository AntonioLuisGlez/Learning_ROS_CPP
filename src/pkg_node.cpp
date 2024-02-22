#include "learning_ros_cpp/pkg_node.h"

#include "learning_ros_cpp/ros_publishers.h"

PkgNode::PkgNode() : nh_(std::make_shared<ros::NodeHandle>("~")), subscriber_(std::make_unique<RosPublishers>(nh_)) {}

void PkgNode::run()
{
    // Configura el bucle principal
    ros::Rate rate(10);
    while (ros::ok()) {
        publisher_->publishMarker();

        rate.sleep();
    }
}
