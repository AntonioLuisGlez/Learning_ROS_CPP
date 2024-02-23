#pragma once

#include <learning_ros_cpp/subscriber_node.h>
#include <ros/ros.h>

class RosSubscribers;

class PkgNode
{
  public:
    PkgNode();
    void run();

  private:
    void odometryThread(float rate);

  private:
    std::shared_ptr<ros::NodeHandle> nh_;
    std::unique_ptr<RosSubscribers>  subscriber_;
};
