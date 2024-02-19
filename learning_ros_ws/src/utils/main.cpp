#include "learning_ros_cpp/markers_node.h"
#include "memory"

int main(int, char**)
{
    // catec::MarkersNode node;
    std::unique_ptr<catec::MarkersNode> node;
    node = std::make_unique<catec::MarkersNode>();

    node->sayHello();

    return 0;
}