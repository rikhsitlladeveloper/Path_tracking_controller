#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
class PathTracker
{
public:
  PathTracker()
  {
    path_sub_ = nh_.subscribe("/gem/global_path", 1, &PathTracker::pathCallback, this);
    pose_sub_ = nh_.subscribe("/gem/base_footprint/odom", 1, &PathTracker::odomCallback, this);
    control_pub_ = nh_.advertise<geometry_msgs::Twist>("/gem/cmd_vel", 1);
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& msg)
  {
    path_= msg->poses;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    current_pose_ = msg->pose.pose;
  }

  void run()
  {
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
      // Calculate the desired steering angle using Pure Pursuit or Stanley
      // ...

      // Publish the control command
      geometry_msgs::Twist control_command;
      control_command.linear.x = 1.0; // Forward velocity
      control_command.angular.z = steering_angle_;
      control_pub_.publish(control_command);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber path_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher control_pub_;
  std::vector<geometry_msgs::PoseStamped> path_;
  geometry_msgs::Pose current_pose_;

  double steering_angle_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_tracker");
  PathTracker path_tracker;
  path_tracker.run();
  return 0;
}
