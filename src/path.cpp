#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

void read_waypoints() {
    std::string filename = "/home/tilla/motion_plan_ws/src/path_tracking_controller/src/wps.csv";
    std::ifstream infile;
    infile.open(filename);
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    std::vector<std::vector<float>> path_points;
    ROS_INFO("Path is opening");

    while(!infile.eof()) {
        std::string line;
        ROS_INFO("%s", line);
        while (getline(infile, line)) {
            std::istringstream iss(line);
            std::vector<float> point;
            std::string val;

            while (getline(iss, val, ',')) {
                point.push_back(std::stof(val));
            }

            path_points.push_back(point);
        }

        infile.close();
    }


    for (auto& point : path_points) {
        pose.pose.position.x = point[0];
        pose.pose.position.y = point[1];
        path.poses.push_back(pose);
        
    
    }

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Path_Publisher");
  ros::NodeHandle n;
  ROS_INFO("Path is generating");

  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/gem/global_path", 1000);
  read_waypoints();
  
  ros::spin();

  return 0;
}

