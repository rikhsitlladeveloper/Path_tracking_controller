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

std::vector<geometry_msgs::PoseStamped> path;
std::vector<double> path_x_points, path_y_points;
ackermann_msgs::AckermannDrive ackermann_msg;
ros::Publisher ackermann_pub;
double distance_x=0 , distance_y = 0, pose_x, pose_y, roll, pitch, yaw, vel_x, vel_y, vel_th, wheelbase = 1.75;
bool plan_recived = false;
void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    for(int i=0; i<msg->poses.size();i++){
      double x = msg->poses[i].pose.position.x;
      double y = msg->poses[i].pose.position.y;
      path_x_points.push_back(x);
      path_y_points.push_back(y); 
    }
    std::cout << "recived path" << std::endl;
    plan_recived = true;
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
    std::cout << "publish_ODOM" << std::endl;
    ros::Rate loop_rate(5);
    // sleep for 200ms
    loop_rate.sleep();

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
    //ros::Rate rate(1);
    //while (ros::ok())
    //{  
      double front_x = wheelbase * cos(yaw) + pose_x;
      double front_y = wheelbase * sin(yaw) + pose_y;
      std::vector<double> dx(path_x_points.size()), dy(path_y_points.size());
      std::transform(path_x_points.begin(), path_x_points.end(), dx.begin(), [front_x](double x){ return front_x - x; });
      std::transform(path_y_points.begin(), path_y_points.end(), dy.begin(), [front_y](double y){ return front_y - y; });
      // find the index of closest point
      int target_index = std::distance(dx.begin(), std::min_element(dx.begin(), dx.end(), [](double a, double b){ return std::hypot(a, b) < std::hypot(b, a); }));
      std::array<double, 3> front_axle_vec_rot_90 = { std::cos(yaw - M_PI / 2.0), std::sin(yaw - M_PI / 2.0) };
      std::array<double, 3> vec_target_2_front = { dx[target_index], dy[target_index] };
      // crosstrack error
      double ef = std::inner_product(vec_target_2_front.begin(), vec_target_2_front.end(), front_axle_vec_rot_90.begin(), 0.0);
      // vehicle heading 
      double theta = yaw;

      // approximate heading of path at (path_x, path_y)
      double path_x      = path_x_points[target_index];
      double path_y      = path_y_points[target_index];
      double path_x_next = path_x_points[target_index+1];
      double path_y_next = path_y_points[target_index+1];
      double theta_p     = std::atan2(path_y_next - path_y, path_x_next - path_x);

      // theta_e is the heading error
      double theta_e = normalize(theta_p - theta);

      double f_vel = std::sqrt(vel_x * vel_x + vel_y * vel_y);
      double delta = theta_e + std::atan2(0.45 * ef, f_vel);

      theta_e  = rad_to_deg(theta_e);

      //ef = round(ef, 3);
      
      // implement constant pure pursuit controller
      ackermann_msg.speed = 2.8;
      ackermann_msg.steering_angle = delta;
      std::cout << "publish" << std::endl;
      ackermann_pub.publish(ackermann_msg);
      // rate.sleep();
      // ros::spin();
   // }
    }
  



int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_tracker");
    ros::NodeHandle nh;
    ros::Subscriber path_sub;
    ros::Subscriber pose_sub;
    ros::Publisher control_pub;
    
    path_sub = nh.subscribe("/gem/global_path", 1, pathCallback);
    pose_sub = nh.subscribe("/gem/base_footprint/odom", 1, odomCallback);
    control_pub = nh.advertise<geometry_msgs::Twist>("/gem/cmd_vel", 1);
    ackermann_pub = nh.advertise<ackermann_msgs::AckermannDrive>("/gem/ackermann_cmd", 1);
    ros::spin();
    ros::Rate rate(10);
    while(plan_recived == true){
        run();
        ros::spin();
        rate.sleep();
    }
    //path_tracker.run();
    
  //ros::MultiThreadedSpinner();
  
  

  return 0;
}



// #include <ros/ros.h>
// #include <std_msgs/String.h>
// #include <boost/thread.hpp>

// // #include <sstream>
// static unsigned int callback1_count = 0;
// static unsigned int callback2_count = 0;
// static unsigned int callback3_count = 0;

// /**
//  * This tutorial demonstrates simple receipt of messages over the ROS system.
//  */
// void subscriberCallback1(const std_msgs::String::ConstPtr& msg)
// {
//     ROS_INFO_STREAM("First subscriber callback: "
//                             << callback1_count
//                             << ", in thread: " << boost::this_thread::get_id());
//     callback1_count++;
//     ros::Rate loop_rate(5);
//     // sleep for 200ms
//     loop_rate.sleep();
// }

// void subscriberCallback2(const std_msgs::String::ConstPtr& msg)
// {
//     ROS_INFO_STREAM("Second subscriber callback: "
//             << callback2_count << ", in thread: "
//             << boost::this_thread::get_id());
//     callback2_count++;
//     ros::Rate loop_rate(5);
//     // sleep for 200ms
//     loop_rate.sleep();
// }

// void subscriberCallback3(const std_msgs::String::ConstPtr& msg)
// {
//     ROS_INFO_STREAM("Third subscriber callback: "
//             << callback3_count << ", in thread: "
//             << boost::this_thread::get_id());
//     callback3_count++;
//     ros::Rate loop_rate(5);
//     // sleep for 200ms
//     loop_rate.sleep();
// }


// /**
//  * This tutorial demonstrates simple sending of messages over the ROS system.
//  */
// int main(int argc, char **argv)
// {

//     ros::init(argc, argv, "talker");
    

//     ros::NodeHandle n;

//     ros::Publisher periodic_pub = n.advertise<std_msgs::String>("/publisher", 1);
//     ros::Subscriber sub1 = n.subscribe<std_msgs::String>("/subscriber",
//                                                          1,
//                                                          subscriberCallback1);
//     ros::Subscriber sub2 = n.subscribe<std_msgs::String>("/subscriber",
//                                                          1,
//                                                          subscriberCallback2);
//     ros::Subscriber sub3 = n.subscribe<std_msgs::String>("/subscriber",
//                                                          1,
//                                                          subscriberCallback3);
//     ros::Rate loop_rate(10);
    
//     /**
//      * A count of how many messages we have sent. This is used to create
//      * a unique string for each message.
//      */
//     int count = 0;
    
//     ROS_INFO_STREAM("Main loop in thread: " << boost::this_thread::get_id());
//     while (ros::ok())
//     {
//         /**
//          * This is a message object. You stuff it with data, and then publish it.
//          */
//         std_msgs::String msg;
        
//         std::stringstream ss;
//         ss << "Periodic Publisher " << count;
//         msg.data = ss.str();
        
//         ROS_INFO_STREAM("" << msg.data.c_str() <<", in thread: "
//             << boost::this_thread::get_id());
   
//         periodic_pub.publish(msg);
        
//         ros::spinOnce();
        
//         loop_rate.sleep();
//         ++count;
//     }
//     return 0;
// }