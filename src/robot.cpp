#include "../include/robot.hpp"
#include "../include/shapeshift.hpp"
#include <std_msgs/msg/string.hpp>

using std::cin;
using std::cout;
using std::endl;
using std::placeholders::_1;

void Robot::subscribe_callback(const ODOM &msg) {
  pose.first = msg.pose.pose.position.x;
  pose.second = msg.pose.pose.position.y;
  theta = asin(msg.pose.pose.orientation.z) * 2;
}

void set_target(double x, double y) {
target_x = x;
target_y = y;
RCLCPP_INFO_STREAM(this->get_logger(),
                    "Going to target: [" << target_x << "," << target_y << "]");
}

void Robot::move(double linear, double angular) {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear;
  msg.angular.z = angular;
  publisher_velocity->publish(msg);
}

void Robot::stop() {
  geometry_msgs::msg::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = 0;
  cmd_vel_msg.angular.z = 0;
  publisher_velocity->publish(cmd_vel_msg);
}

void Robot::process_callback() {
  std::pair<double, double> target{target_x, target_y};
  double K_linear = 0.1;
  double K_angular = 1.0;
  double distance = abs(
      sqrt(pow((target_x - (pose.first)), 2) + pow((target_y - (pose.second)), 2)));
  double linear_speed = distance * K_linear;
  double desired_angle_target = atan2(target_y - pose.second, target_x - pose.first);
  double angular_speed = (desired_angle_target - theta) * K_angular;

  move(linear_speed, angular_speed);

  if (distance < 0.1) {
    stop();
    RCLCPP_INFO(this->get_logger(), "I have reached the target");
  }
}