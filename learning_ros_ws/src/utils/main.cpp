#include <memory>

#include "learning_ros_cpp/markers_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "markers_node");

    std::unique_ptr<catec::MarkersNode> node;
    node = std::make_unique<catec::MarkersNode>();

    node->init();

    ros::spin();

    return 0;
}
