#pragma once

#include <ros/ros.h>

#include <memory>

#include "learning_ros_cpp/ros_publishers.h"

class PkgNode
{
  public:
    PkgNode();
    void run();

  private:
    std::shared_ptr<ros::NodeHandle> nh_;
    std::unique_ptr<RosPublishers>   publisher_;
};
