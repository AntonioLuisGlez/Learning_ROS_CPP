#include <ros/ros.h>

#include "learning_ros_cpp/pkg_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pkg_main"); // Inicializa el nodo de ROS

    // Crea un puntero único para el nodo manejador del paquete y ejecuta el nodo
    std::unique_ptr<PkgNode> pkg_node = std::make_unique<PkgNode>();
    pkg_node->run();

    return 0;
}
