#include "learning_ros_cpp/markers_node.h"

namespace catec {

MarkersNode::MarkersNode() : nh_("~")
{
    /// \note: Here define and create the subscribers and publishers

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("Marker_for_sensor1", 1);
}

MarkersNode::~MarkersNode() {}

void MarkersNode::init()
{
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

void MarkersNode::step()
{
    marker_pub_.publish(marker_);

    // ros::Rate rate(10); // Frecuencia de publicación en Hz

    // while (ros::ok()) {
    //     /* Cambia la posición del marcador
    //     marker_.pose.position.x += 0.1;
    //     marker_.pose.position.y += 0.1;
    //     */

    //     // Publica el marcador
    //     marker_pub_.publish(marker_);

    //     // Espera hasta la próxima iteración
    //     rate.sleep();
    // }
}

} // namespace catec
