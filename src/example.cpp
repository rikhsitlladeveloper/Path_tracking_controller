#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <tf/transformations.h>
#include <fstream>
#include <vector>
#include <cmath>

class Stanley
{
public:
    Stanley() : rate(20), wheelbase(1.75)
    {
        readWaypoints();

        ackermannMsg.steering_angle_velocity = 0.0;
        ackermannMsg.acceleration = 0.0;
        ackermannMsg.jerk = 0.0;
        ackermannMsg.speed = 0.0;
        ackermannMsg.steering_angle = 0.0;

        ackermannPub = nh.advertise<ackermann_msgs::AckermannDrive>("/gem/ackermann_cmd", 1);
    }

    void readWaypoints()
    {
        std::string path = ros::package::getPath("package_name") + "/waypoints/wps.csv";
        std::ifstream file(path);

        if (!file.is_open()) {
            ROS_ERROR_STREAM("Unable to open file: " << path);
            return;
        }

        std::vector<double> x, y, yaw;
        std::string line;
        while (std::getline(file, line)) {
            double point_x, point_y, point_yaw;
            std::istringstream ss(line);
            ss >> point_x >> point_y >> point_yaw;
            x.push_back(point_x);
            y.push_back(point_y);
            yaw.push_back(point_yaw);
        }

        pathPointsX = x;
        pathPointsY = y;
        pathPointsYaw = yaw;

        distArr.resize(pathPointsX.size());
    }
       void getGemState() {
        ros::ServiceClient client = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        gazebo_msgs::GetModelState srv;
        srv.request.model_name = "gem";
        if (client.call(srv)) {
            double x = srv.response.pose.position.x;
            double y = srv.response.pose.position.y;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(srv.response.pose.orientation, quat);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            double x_dot = srv.response.twist.linear.x;
            double y_dot = srv.response.twist.linear.y;
            state_ = { x, y, x_dot, y_dot, yaw };
        } else {
            ROS_ERROR("Failed to call service /gazebo/get_model_state");
        }
    }

    double pi2pi(double angle) {
        if (angle > M_PI) {
            return angle - 2.0 * M_PI;
        }
        if (angle < -M_PI) {
            return angle + 2.0 * M_PI;
        }
        return angle;

    }
    void startStanley()
{
    while (ros::ok())
    {
        // get current position and orientation in the world frame
        // reference point is located at the center of rear axle
        double curr_x, curr_y, curr_x_dot, curr_y_dot, curr_yaw;
        std::tie(curr_x, curr_y, curr_x_dot, curr_y_dot, curr_yaw) = getGemState();

        // reference point is located at the center of frontal axle
        double front_x = wheelbase*cos(curr_yaw) + curr_x;
        double front_y = wheelbase*sin(curr_yaw) + curr_y;

        // find the closest point
        std::vector<double> dx(path_points_x.size()), dy(path_points_y.size());
        std::transform(path_points_x.begin(), path_points_x.end(), dx.begin(), [front_x](double x) { return front_x - x; });
        std::transform(path_points_y.begin(), path_points_y.end(), dy.begin(), [front_y](double y) { return front_y - y; });
        int target_index = std::distance(std::hypot(dx, dy).begin(), std::min_element(std::hypot(dx, dy).begin(), std::hypot(dx, dy).end()));

        std::array<double, 2> front_axle_vec_rot_90{cos(curr_yaw - M_PI / 2.0), sin(curr_yaw - M_PI / 2.0)};
        std::array<double, 2> vec_target_2_front{dx[target_index], dy[target_index]};

        // crosstrack error
        double ef = std::inner_product(vec_target_2_front.begin(), vec_target_2_front.end(), front_axle_vec_rot_90.begin(), 0.0);

        // vehicle heading 
        double theta = curr_yaw;

        // approximate heading of path at (path_x, path_y)
        double path_x = path_points_x[target_index];
        double path_y = path_points_y[target_index];
        double path_x_next = path_points_x[target_index+1];
        double path_y_next = path_points_y[target_index+1];
        double theta_p = std::atan2(path_y_next-path_y, path_x_next-path_x);

        // theta_e is the heading error
        double theta_e = pi2pi(theta_p-theta);

        double f_vel = std::sqrt(curr_x_dot*curr_x_dot + curr_y_dot*curr_y_dot);

        double delta = theta_e + std::atan2(0.45 * ef, f_vel);

        theta_e = rad2deg(theta_e);
        ef = round(ef,3);
        std::cout << "Crosstrack Error: " << ef << ", Heading Error: " << theta_e << std::endl;

        // implement constant pure pursuit controller
        ackermann_msg.speed = 2.8;
        ackermann_msg.steering_angle = delta;
        ackermann_pub.publish(ackermann_msg);

        rate.sleep();
    }
}

std::tuple<double, double, double, double, double> getGemState()
{
    ros::service::waitForService("/gazebo/get_model_state");

    gazebo_msgs::GetModelState srv;
    srv.request.model_name = "gem";
    srv.request.relative_entity_name = "world";

    try
    {
        gazebo_client.call(srv);
    }
    catch (const std::exception& e)
    {
        ROS_INFO("Service did not process request: %s", e.what());
    }

    double x = srv.response.pose.position.x;
    double y = srv.response.pose.position.y;

    tf2

private:
    ros::NodeHandle nh;
    ros::Rate rate;
    double wheelbase;
    std::vector<double> pathPointsX;
    std::vector<double> pathPointsY;
    std::vector<double> pathPointsYaw;
    std::vector<double> distArr;
    ackermann_msgs::AckermannDrive ackermannMsg;
    ros::Publisher ackermannPub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stanley_node");
    Stanley stanley;
    ros::spin();
    return 0;
}



void start_stanley()
{
    while (ros::ok())
    {
        // get current position and orientation in the world frame
        // reference point is located at the center of rear axle
        double curr_x, curr_y, curr_x_dot, curr_y_dot, curr_yaw;
        std::tie(curr_x, curr_y, curr_x_dot, curr_y_dot, curr_yaw) = get_gem_state();

        // reference point is located at the center of frontal axle
        double front_x = wheelbase * std::cos(curr_yaw) + curr_x;
        double front_y = wheelbase * std::sin(curr_yaw) + curr_y;

        // find the closest point
        std::vector<double> dx(path_points_x.size()), dy(path_points_y.size());
        std::transform(path_points_x.begin(), path_points_x.end(), dx.begin(), [front_x](double x){ return front_x - x; });
        std::transform(path_points_y.begin(), path_points_y.end(), dy.begin(), [front_y](double y){ return front_y - y; });

        // find the index of closest point
        int target_index = std::distance(dx.begin(), std::min_element(dx.begin(), dx.end(), [](double a, double b){ return std::hypot(a, b) < std::hypot(b, a); }));

        std::array<double, 2> front_axle_vec_rot_90 = { std::cos(curr_yaw - M_PI / 2.0), std::sin(curr_yaw - M_PI / 2.0) };
        std::array<double, 2> vec_target_2_front = { dx[target_index], dy[target_index] };

        // crosstrack error
        double ef = std::inner_product(vec_target_2_front.begin(), vec_target_2_front.end(), front_axle_vec_rot_90.begin(), 0.0);

        // vehicle heading 
        double theta = curr_yaw;

        // approximate heading of path at (path_x, path_y)
        double path_x      = path_points_x[target_index];
        double path_y      = path_points_y[target_index];
        double path_x_next = path_points_x[target_index+1];
        double path_y_next = path_points_y[target_index+1];
        double theta_p     = std::atan2(path_y_next - path_y, path_x_next - path_x);

        // theta_e is the heading error
        double theta_e = pi_2_pi(theta_p - theta);

        double f_vel = std::sqrt(curr_x_dot * curr_x_dot + curr_y_dot * curr_y_dot);

        double delta = theta_e + std::atan2(0.45 * ef, f_vel);

        theta_e  = rad2deg(theta_e);

        ef = round(ef, 3);
        ROS_INFO_STREAM("Crosstrack Error: " << ef << ", Heading Error: " << theta_e);

        // implement constant pure pursuit controller
        ackermann_msg.speed          = 2.8;
        ackermann_msg.steering_angle = delta;
        ackermann_pub.publish(ackermann_msg);

        rate.sleep();
    }
}