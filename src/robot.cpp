/**
 * @file robot.cpp
 *
 * @author  Phase 1 - Shivam Sehgal (ssehgal7@umd.edu) - Driver,
 *                    Patrik Pordi (ppordi@umd.edu) - Navigator,
 *                    Darshit Desai (darshit@umd.edu) - Code designer
 *          Phase 2 - Shivam Sehgal (ssehgal7@umd.edu) - Code designer,
 *                    Patrik Pordi (ppordi@umd.edu) - Driver,
 *                    Darshit Desai (darshit@umd.edu) - Navigator
 * @brief Implementation of Robot class
 * @version 0.1
 * @date 2023-12-19
 * @copyright Copyright (c) 2023 Darshit Desai, Patrik Pordi, Shivam Sehgal
 * This code is licensed under the MIT License. Please see the
 * accompanying LICENSE file for the full text of the license.
 */
#include "../include/shape_shifters/robot.hpp"

#include <std_msgs/msg/string.hpp>

/**
 * @brief Implementation of the odom topic callback function
 *
 */
void robot::odom_callback(
    std::shared_ptr<nav_msgs::msg::Odometry_<std::allocator<void>>> msg) {
  // Get the current position of the robot
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  // Get the current orientation of the robot
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  // Convert the quaternion to euler angles
  tf2::Quaternion q(qx, qy, qz, qw);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  // Update the heading angle
  setHeadingAngle(yaw);
  // Update the current position using setter
  setCurrentPosition(x, y);
}
/**
 * @brief Implementation of the velocity calculation function
 * @return std::pair<double, double>
 *
 */
std::pair<double, double> robot::velocity_calculation() {
  // Define the proportional gain for linear and angular velocity
  double kp_linear = 0.1;
  double kp_angular = 1.0;
  // Calculate the distance to the target
  double distance = sqrt(pow(target.first - current_position.first, 2) +
                         pow(target.second - current_position.second, 2));
  // Calculate the angle to the target
  double angle = atan2(target.second - current_position.second,
                       target.first - current_position.first);
  // Calculate the difference between the current angle and the target angle
  double angle_diff = angle - headingAngle;
  // Calculate the angular velocity
  double angular_velocity = angle_diff * kp_angular;
  // Calculate the linear velocity
  double linear_velocity = distance * kp_linear;
  return {linear_velocity, angular_velocity};
}
/**
 * @brief Implementation of the get_goal_distance function
 *
 * @return double
 */
double robot::get_goal_distance() {
  double distance = sqrt(pow(target.first - current_position.first, 2) +
                         pow(target.second - current_position.second, 2));
  return distance;
}
/**
 * @brief Implementation of the velocity callback function
 *
 */
void robot::velocity_callback() {
  auto [linear_velocity, angular_velocity] = velocity_calculation();
  // Publish the velocity
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear_velocity;
  msg.angular.z = angular_velocity;
  publisher_->publish(msg);
  if (get_goal_distance() < 0.25) {
    // Stop the robot
    msg.linear.x = 0;
    msg.angular.z = 0;
    publisher_->publish(msg);
    // Print a message
    RCLCPP_INFO_STREAM(rclcpp::get_logger("tbot_walker"),
                       "Robot reached the target!");
  }
}
/**
 * @brief Implementation of the setTarget function
 * @param x
 * @param y
 */
void robot::setTarget(double x, double y) {
  target.first = x;
  target.second = y;
}

/**
 * @brief Implementation of the get target function
 * @return std::pair<double, double>
 */
std::pair<double, double> robot::getTarget() { return target; }
/**
 * @brief Implementation of the setCurrentPosition function
 * @param x
 * @param y
 */
void robot::setCurrentPosition(double x, double y) {
  current_position.first = x;
  current_position.second = y;
}

/**
 * @brief Implementation of the get Heading Angle function
 * @return double
 */
double robot::getHeadingAngle() { return headingAngle; }

/**
 * @brief Implementation of the set Heading Angle function
 *
 */
void robot::setHeadingAngle(double angle) { headingAngle = angle; }

/**
 * @brief Implementation of the get current position function
 * @return std::pair<double, double>
 */
std::pair<double, double> robot::getCurrentPosition() {
  return current_position;
}
