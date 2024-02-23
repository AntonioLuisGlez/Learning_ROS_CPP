#include "learning_ros_cpp/pkg_node.h"

PkgNode::PkgNode() : nh_(std::make_shared<ros::NodeHandle>("~")), subscriber_(std::make_unique<RosSubscribers>(nh_)) {}

void PkgNode::run()
{
    // Configura el bucle principal
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce(); // Procesa los callbacks pendientes
        rate.sleep();
    }
}
