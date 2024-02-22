#include "learning_ros_cpp/ros_publishers.h"

#include <geometry_msgs/TransformStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

RosPublishers::RosPublishers(std::shared_ptr<ros::NodeHandle> nh) : nh_(*nh)
{
    // Inicializa los publicadores ROS
    std::string marker_topic, pointcloud_topic;
    nh_.param<std::string>("marker_topic", marker_topic, "marker_for_sensor1");

    marker_pub_      = nh_.advertise<visualization_msgs::Marker>(marker_topic, 1);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_topic", 1);

    // Recupera los parámetros del nodo manejador del paquete
    nh_.param("radius", radius_, 1.0);
    nh_.param("angular_velocity", angular_velocity_, 0.1);
    nh_.param("frame_id", frame_id_, std::string("odom"));
    nh_.param("child_frame_id", child_frame_id_, std::string("base_link"));
}

void RosPublishers::publishMarker()
{
    visualization_msgs::Marker marker_msg;
    // Set marker properties
    marker_msg.header.frame_id    = "sensor1_frame";
    marker_msg.header.stamp       = ros::Time::now();
    marker_msg.ns                 = "basic_shapes";
    marker_msg.id                 = 0;
    marker_msg.type               = visualization_msgs::Marker::SPHERE;
    marker_msg.action             = visualization_msgs::Marker::ADD;
    marker_msg.pose.position.x    = 0;
    marker_msg.pose.position.y    = 0;
    marker_msg.pose.position.z    = 0;
    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x            = 0.5;
    marker_msg.scale.y            = 0.5;
    marker_msg.scale.z            = 0.5;
    marker_msg.color.r            = 1.0f;
    marker_msg.color.g            = 0.0f;
    marker_msg.color.b            = 0.0f;
    marker_msg.color.a            = 1.0;
    marker_msg.lifetime           = ros::Duration();

    marker_pub_.publish(marker_msg);
}

void RosPublishers::publishPointCloud()
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

    // Create a PointCloud2 message and convert the PointCloud to it
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "sensor2_frame";
    cloud_msg.header.stamp    = ros::Time::now();
    // Publish the PointCloud2 message
    point_cloud_pub_.publish(cloud_msg);
}

void RosPublishers::publishCircularPath()
{
    // Calcula la posición en la trayectoria circular
    double current_angle = angular_velocity_ * ros::Time::now().toSec();
    double x             = radius_ * cos(current_angle);
    double y             = radius_ * sin(current_angle);

    // Publica la transformación entre frames
    geometry_msgs::TransformStamped transformStamped;
    // Configura la transformación
    transformStamped.header.stamp            = ros::Time::now();
    transformStamped.header.frame_id         = frame_id_;
    transformStamped.child_frame_id          = child_frame_id_;
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x    = 0.0;
    transformStamped.transform.rotation.y    = 0.0;
    transformStamped.transform.rotation.z    = 0.0;
    transformStamped.transform.rotation.w    = 1.0;

    // Publica la transformación
    tf_broadcaster_.sendTransform(transformStamped);
}
