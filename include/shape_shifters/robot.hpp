/**
 * @file robot.hpp
 *
 * @author  Phase 1 - Shivam Sehgal (ssehgal7@umd.edu) - Driver,
 *                    Patrik Pordi (ppordi@umd.edu) - Navigator,
 *                    Darshit Desai (darshit@umd.edu) - Code designer
 *          Phase 2 - Shivam Sehgal (ssehgal7@umd.edu) - Code designer,
 *                    Patrik Pordi (ppordi@umd.edu) - Driver,
 *                    Darshit Desai (darshit@umd.edu) - Navigator
 * @brief Robot class
 * @version 0.1
 * @date 2023-12-19
 *
 * @copyright Copyright (c) 2023
 *
 *
 * @copyright Copyright (c) 2023 Darshit Desai, Patrik Pordi, Shivam Sehgal
 * This code is licensed under the MIT License. Please see the
 * accompanying LICENSE file for the full text of the license.
 *
 */

#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
/**
 * @brief Class for the robot node
 *
 */
class robot : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Walker object
   * @param name
   * @param node_name
   *
   */
  robot(std::string name, std::string node_name)
      : Node(node_name), namespace_robot{name}, target{0, 0} {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("tbot_walker"),
                       "Setting up Robot Node...");
    // Create a publisher for the cmd_vel topic
    auto velocity_topic = "/" + namespace_robot + "/cmd_vel";
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>(velocity_topic, 10);
    // Create a subscription to the odom topic
    auto odom_topic = "/" + namespace_robot + "/odom";
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        std::bind(&robot::odom_callback, this, std::placeholders::_1));
    // Create a timer to publish the velocity
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&robot::velocity_callback, this));
  }
  /**
   * @brief Subscribe to the odom topic using callback function
   * @param msg
   */
  void odom_callback(
      std::shared_ptr<nav_msgs::msg::Odometry_<std::allocator<void>>> msg);
  /**
   * @brief Calculate the velocity and publish it
   *
   */
  void velocity_callback();
  /**
   * @brief Setters for the target
   * @param x
   * @param y
   */
  void setTarget(double x, double y);
  /**
   * @brief Function to calculate the velocity
   *
   * @return std::pair<double, double>
   */
  std::pair<double, double> velocity_calculation();
  /**
   * @brief Function to calculate the distance to the target
   *
   * @return double
   */
  double get_goal_distance();
  /**
   * @brief Setter for the current position
   * @param x
   * @param y
   *
   */
  void setCurrentPosition(double x, double y);
  /**
   * @brief Getter for the current position
   *
   * @return std::pair<double, double>
   */
  std::pair<double, double> getCurrentPosition();
  /**
   * @brief Getter for the target
   * @return std::pair<double, double>
   *
   */
  std::pair<double, double> getTarget();
  /**
   * @brief Getter for the heading angle
   *
   * @return double
   */
  double getHeadingAngle();
  /**
   * @brief Setter for the heading angle
   *
   */
  void setHeadingAngle(double angle);

 private:
  std::string namespace_robot;
  std::pair<double, double> target;
  std::pair<double, double> current_position;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  double headingAngle;
};
