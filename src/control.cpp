#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "std_msgs/Float64.h"
#include <cmath>
#include <numeric>
#include <chrono>
#include <thread>
class PathTracker
{
public:
  PathTracker()
  {
    
    path_sub_ = nh_.subscribe("/gem/global_path", 1, &PathTracker::pathCallback, this);
    pose_sub_ = nh_.subscribe("/gem/base_footprint/odom", 1, &PathTracker::odomCallback, this);
    control_pub_ = nh_.advertise<geometry_msgs::Twist>("/gem/cmd_vel", 1);
    ackermann_pub = nh_.advertise<ackermann_msgs::AckermannDrive>("/gem/ackermann_cmd", 1);
  } 

  void pathCallback(const nav_msgs::Path::ConstPtr& msg)
  {
    for(int i=0; i<msg->poses.size();i++){
      double x = msg->poses[i].pose.position.x;
      double y = msg->poses[i].pose.position.y;
      path_x_points.push_back(x);
      path_y_points.push_back(y); 
    }
    path_recived = true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    pose_x = msg->pose.pose.position.x;
    pose_y = msg->pose.pose.position.y;
    tf2::Quaternion quat(msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z,
                         msg->pose.pose.orientation.w);
    tf2::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw);
    vel_x = msg->twist.twist.linear.x;
    vel_y = msg->twist.twist.linear.y;
    vel_th = msg->twist.twist.angular.z; 
    if(path_recived){
      run();
    }

  }

  // Normalizing angle
  double normalize(double angle)
  {
    if (angle > M_PI)
      return angle - 2 * M_PI;
    
    if (angle < M_PI)
      return angle + 2 * M_PI;
    
    return angle;
  }

  double rad_to_deg(double radians)
  {
    return radians * 180 / M_PI;
  }

  void run()
  {         
      double front_x = wheelbase * cos(yaw) + pose_x;
      double front_y = wheelbase * sin(yaw) + pose_y;
      std::cout << "front_x: " << front_x << std::endl;
      std::cout << "front_y: " << front_y << std::endl;
      std::cout << "path_x_size: " << path_x_points.size() << std::endl;
      std::cout << "path_y_size: " << path_y_points.size() << std::endl;
      
      std::vector<double> dx, dy, dxy;
      
      for(double x : path_x_points){
        dx.push_back(front_x - x);
      }

      for(double y : path_y_points){
        dy.push_back(front_y - y);  
      }
      

      for(int i=0;i<dx.size();i++){
        double xy_hypot = std::hypot(dx[i],dy[i]); 
        dxy.push_back(xy_hypot);
      } 
      
      std::cout << "dx_size: " << dx.size() << std::endl;
      std::cout << "dy_size: " << dy.size() << std::endl;

      //std::transform(path_y_points.begin(), path_y_points.end(), dy.begin(), [front_y](double y){ return front_y - y; });
      // find the index of closest point
      // int target_index = std::distance(dx.begin(), std::min_element(dx.begin(), dx.end(), [](double a, double b){ return std::hypot(a, b) < std::hypot(b, a); }));
      //int target_index = std::distance(dxy.begin(), std::min_element(dxy.begin(), dxy.end()));
      //int target_index = std::distance(dx, std::min_element(dx, dx + dx.size(), [dy](double a, double b) {
      //return std::hypot(a, dy[std::distance(dx, &a)]) < std::hypot(b, dy[std::distance(dx, &b)]);}));
      target_index = std::min_element(dx, dy);
      // double min_dist = dxy[0];
      // for(int i=0; i<dxy.size();i++){
      //   if(dxy[i]< min_dist){
      //     min_dist = dxy[i];
          
      //     int target_index = i;
      //   }
      std::cout << "index ;"<< i <<" "<< dxy[i] << std::endl;
      
      std::cout << "index:" << target_index << std::endl;
      std::array<double, 3> front_axle_vec_rot_90 = { std::cos(yaw - M_PI / 2.0), std::sin(yaw - M_PI / 2.0) };
      std::array<double, 3> vec_target_2_front = { dx[target_index], dy[target_index] };
      // crosstrack error
      double ef = std::inner_product(vec_target_2_front.begin(), vec_target_2_front.end(), front_axle_vec_rot_90.begin(), 0.0);
      // vehicle heading 
      double theta = yaw;
      // std::cout << "theta :" << theta << std::endl;
      // approximate heading of path at (path_x, path_y)

      double path_x      = path_x_points[target_index];
      double path_y      = path_y_points[target_index];
      double path_x_next = path_x_points[target_index+1];
      double path_y_next = path_y_points[target_index+1];
      if (init == false){
        relative_x = pose_x - path_x_next;
        relative_y = pose_y - path_y_next;
        init = true;
      }
      pose_x = pose_x + relative_x;
      pose_y = pose_y + relative_y;
      std::cout << "path: " << path_y_next << path_y << path_x_next << path_x <<std::endl;
      //double theta_p     = std::atan((path_y_next - path_y) / (path_x_next - path_x));
      double theta_p     = std::atan((path_y_next - pose_y) / (path_x_next - pose_x));

      // std::cout << "theta_p:" << theta_p << std::endl;
      // theta_e is the heading error
      double theta_e = theta_p + theta;
      std::cout << "theta_e: " << theta_e << std::endl;
      double f_vel = std::sqrt(vel_x * vel_x + vel_y * vel_y);
      double delta = theta_e + std::atan(0.45 * ef/ f_vel);

      // theta_e  = rad_to_deg(theta_e);

      //ef = round(ef, 3);
      std::cout << "delta: " << delta << std::endl;
      // implement constant pure pursuit controller
      ackermann_msg.speed = 5;
      ackermann_msg.steering_angle = -theta_e;
      //std::cout << "publish" << std::endl;
      // ackermann_pub.publish(ackermann_msg);
    }
  

public:
  ros::NodeHandle nh_;
  ros::Subscriber path_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher control_pub_;
  std::vector<geometry_msgs::PoseStamped> path_;
  std::vector<double> path_x_points, path_y_points;
  ackermann_msgs::AckermannDrive ackermann_msg;
  ros::Publisher ackermann_pub;
  double distance_x=0 , distance_y = 0, pose_x, pose_y, roll, pitch, yaw, vel_x, vel_y, vel_th, wheelbase = 1.75, relative_x, relative_y;
  bool path_recived = false, init = false;
  int target_index;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_tracker");
  PathTracker path_tracker;
  // ros::Rate rate(10);
  //path_tracker.run();
  // while (ros::ok())
  // {  
  //   path_tracker.run();
        
  //   rate.sleep();  
  // }
  ros::spin();

  return 0;
}
