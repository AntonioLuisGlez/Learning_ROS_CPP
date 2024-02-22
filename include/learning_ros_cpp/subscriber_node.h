#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class RosSubscribers
{
  public:
    RosSubscribers(const std::shared_ptr<ros::NodeHandle>& nh);
    ~RosSubscribers();

    nav_msgs::Odometry get_current_odometry();

  private:
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
    void quaternionCb(const geometry_msgs::Quaternion::ConstPtr& msg);

  private:
    std::shared_ptr<ros::NodeHandle> _nh;
    std::mutex                       _mtx;

    ros::Subscriber _odometry_sub;
    ros::Subscriber _orientation_sub;

    nav_msgs::Odometry _current_odometry;
};
