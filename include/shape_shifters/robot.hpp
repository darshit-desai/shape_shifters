#ifndef INCLUDE_ROBOT_HPP_
#define INCLUDE_ROBOT_HPP_

#include <string>
#include <utility>
#include <chrono>
#include <iostream>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;

/**
 * @brief Class for the Robot node
 *
 */
class Robot : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Robot object
   *
   * @param node_name  name of the node
   * @param robot_name  name of the robot
   */
  Robot(std::string const &node_name, std::string const &robot_name)
      : Node(node_name), robot_ns{robot_name}, target_x{0.0}, target_y{0.0} {
    callback_grp = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto cmd_vel_topic = "/" + robot_ns + "/cmd_vel";
    auto pose_topic = "/" + robot_ns + "/odom";

    // creates publisher to publish /cmd_vel topic
    publisher_velocity = this->create_publisher<TWIST>(cmd_vel_topic, 1);

    auto subCallback0 = std::bind(&RoomBa::subscribe_callback, this, _1);
    subscription_pose =
        this->create_subscription<ODOM>(pose_topic, 1, subCallback0);

    auto processCallback = std::bind(&RoomBa::process_callback, this);
    timer_ = this->create_wall_timer(100ms, processCallback, callback_grp);
  }

  /**
   * @brief Set the goal coordinates for the robot
   *
   * @param x  x coordinate of the goal
   * @param y  y coordinate of the goal
   */
  void set_target(double x, double y);

 private:
  double target_x;       // X coordinate of the target/goal
  double target_y;       // Y coordinate of the target/goal
  double headingAngle;   // Heading angle of the robot
  std::string robot_ns;  // Namespace of the robot

  // ROS 2 Node components
  rclcpp::Subscription<ODOM>::SharedPtr subscription_1;  // Odometry subscription
  rclcpp::Publisher<TWIST>::SharedPtr publisher_1;       // Velocity publisher
  rclcpp::TimerBase::SharedPtr timer_;                    // Timer for periodic callbacks
  rclcpp::CallbackGroup::SharedPtr callback_grp;         // Callback group for node components

  /**
   * @brief Subscribe callback function to get the pose of the robot
   *
   * @param msg  message of type ODOM
   */
  void subscribe_callback(const ODOM &msg);

  /**
   * @brief Process callback function to publish velocity to the robot
   *
   */
  void process_callback();

  /**
   * @brief Move the robot by publishing velocity
   *
   * @param linear  linear velocity
   * @param angular  angular velocity
   */
  void move(double linear, double angular);

  /**
   * @brief Stop the robot by publishing 0 velocity
   *
   */
  void stop();
};

#endif  // INCLUDE_ROBOT_HPP_
