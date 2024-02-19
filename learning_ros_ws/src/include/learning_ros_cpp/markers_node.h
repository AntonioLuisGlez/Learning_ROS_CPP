#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace catec {

class MarkersNode
{
  public:
    MarkersNode();
    ~MarkersNode();

    void init();

  private:
    ros::NodeHandle            nh_;
    ros::Publisher             marker_pub_;
    visualization_msgs::Marker marker_;
};

} // namespace catec
