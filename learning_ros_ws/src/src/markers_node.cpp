#include "learning_ros_cpp/markers_node.h"

#include "iostream"

namespace catec {
MarkersNode::MarkersNode()
{
    std::cout << "Hola :). Soy el constructor de la clase" << std::endl;
}
MarkersNode::~MarkersNode()
{
    std::cout << "Chao :). Soy el destructor de la clase" << std::endl;
}

void MarkersNode::sayHello()
{
    std::cout << "Hello!" << std::endl;
}

} // namespace catec