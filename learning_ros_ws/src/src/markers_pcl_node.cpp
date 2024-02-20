#include "learning_ros_cpp/markers_pcl_node.h"

namespace catec {

// Constructor for MarkersNode, initializing nh_ with private namespace
MarkersNode::MarkersNode() : nh_("~")
{
    /// \note: Here define and create the subscribers and publishers
    // Advertise marker publisher for sensor1
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("Marker_for_sensor1", 1);
}

// Empty destructor for MarkersNode
MarkersNode::~MarkersNode() {}

// Initialize the marker properties
void MarkersNode::init()
{
    // Set marker properties
    marker_.header.frame_id    = "sensor1_frame";
    marker_.header.stamp       = ros::Time::now();
    marker_.ns                 = "basic_shapes";
    marker_.id                 = 0;
    marker_.type               = visualization_msgs::Marker::SPHERE;
    marker_.action             = visualization_msgs::Marker::ADD;
    marker_.pose.position.x    = 0;
    marker_.pose.position.y    = 0;
    marker_.pose.position.z    = 0;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x            = 0.5;
    marker_.scale.y            = 0.5;
    marker_.scale.z            = 0.5;
    marker_.color.r            = 1.0f;
    marker_.color.g            = 0.0f;
    marker_.color.b            = 0.0f;
    marker_.color.a            = 1.0;
    marker_.lifetime           = ros::Duration();
}

// Publish the marker
void MarkersNode::step()
{
    marker_pub_.publish(marker_);
}

// Constructor for PointCloudGenerator, initializing nh_ with private namespace
PointCloudGenerator::PointCloudGenerator() : nh_("~")
{
    /// \note: Here define and create the subscribers and publishers
    // Advertise point cloud publisher
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_topic", 1);
}

// Generate and publish the point cloud
void PointCloudGenerator::step()
{
    // Generate the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloud();
    // Publish the generated point cloud
    publishPointCloud(cloud);
}

// Function to generate a point cloud with a grid of points
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudGenerator::generatePointCloud()
{
    // Create a new PointCloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Populate the point cloud with a grid of points
    for (float x = -1.0; x <= 1.0; x += 0.1) {
        for (float y = -1.0; y <= 1.0; y += 0.1) {
            // Define a new point
            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = 0.0;
            // Add the point to the point cloud
            cloud->points.push_back(point);
        }
    }

    cloud->width  = cloud->points.size();
    cloud->height = 1;

    return cloud;
}

// Publish the generated point cloud
void PointCloudGenerator::publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // Create a PointCloud2 message and convert the PointCloud to it
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "sensor2_frame";
    cloud_msg.header.stamp    = ros::Time::now();
    // Publish the PointCloud2 message
    point_cloud_pub_.publish(cloud_msg);
}

} // namespace catec
