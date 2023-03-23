#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include <thread>
   
int main(int argc, char **argv)
{
    int j = 0;
    ros::init(argc, argv, "Path_Publisher");
    ros::NodeHandle n;
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/gem/global_path", 1000000);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::vector<std::vector<float>> data;
    std::string line;
    ROS_INFO("Path is generating");

    std::string filename = "/home/tilla/motion_plan_ws/src/path_tracking_controller/src/wps.csv";
    std::ifstream infile;
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    path.header.frame_id = "base_link";
    path.header.stamp = ros::Time::now();

    std::vector<std::vector<float>> path_points;
    std::ifstream file(filename);

    if (!file.is_open()) {
    std::cerr << "Failed to open file" << std::endl;
    }


    while (std::getline(file, line)) {
    std::vector<float> row;
    size_t pos = 0;
    while (pos < line.size()) {
    size_t endpos = line.find(',', pos);
    if (endpos == std::string::npos) {
        endpos = line.size();
    }
    std::string cell = line.substr(pos, endpos - pos);
    row.push_back(std::stof(cell));
    pos = endpos + 1;
    }
    data.push_back(row);
    }

    for (const auto& row : data) {
        pose.pose.position.x = row[0];
        pose.pose.position.y = row[1];
        pose.header.seq = j;
        pose.header.frame_id = "base_link";
        pose.header.stamp = ros::Time::now();
        j++;
        path.poses.push_back(pose);
    }
    path_pub.publish(path);

    file.close();
    ros::spin();

    return 0;
}

