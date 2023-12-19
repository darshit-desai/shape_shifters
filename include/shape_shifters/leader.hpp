#ifndef INCLUDE_LEADER_HPP_
#define INCLUDE_LEADER_HPP_

#include <vector>
#include <memory>
#include "../include/robot.hpp"

using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;

/**
 * @brief Leader class to control the robots and spawn nodes
 *
 */
class Leader : public rclcpp::Node {
 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<TWIST>::SharedPtr publisher_;
  std::vector<std::shared_ptr<Robot>> robot_array;
 /**
   * @brief Construct a new Leader object
   *
   * @param robot_array  Vector of robot objects
   */
 explicit Leader(std::vector<std::shared_ptr<Robot>> const &robot_array);
  
};

#endif  // INCLUDE_LEADER_HPP_
