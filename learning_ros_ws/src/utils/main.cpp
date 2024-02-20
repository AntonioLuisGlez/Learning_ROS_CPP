#include <ros/ros.h> // Include ROS library

#include <memory> // Include memory library for smart pointers

#include "learning_ros_cpp/markers_pcl_node.h" // Include header file for MarkersNode and PointCloudGenerator classes

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_and_pointcloud_node"); // Initialize the ROS node with a specific node name

    // Create a unique pointer for MarkersNode
    std::unique_ptr<catec::MarkersNode> markers_node = std::make_unique<catec::MarkersNode>();

    // Initialize the MarkersNode
    markers_node->init();

    // Create a unique pointer for PointCloudGenerator
    std::unique_ptr<catec::PointCloudGenerator> point_cloud_generator = std::make_unique<catec::PointCloudGenerator>();

    // Set the publishing rate to 10 Hz
    ros::Rate rate(10);

    // Main loop
    while (ros::ok()) {
        markers_node->step();          // Perform a step in MarkersNode
        point_cloud_generator->step(); // Perform a step in PointCloudGenerator

        // Wait until the next iteration based on the rate
        rate.sleep();
    }

    // Uncomment if using ROS callbacks to process messages
    // ros::spin();

    return 0; // Exit the main function
}
