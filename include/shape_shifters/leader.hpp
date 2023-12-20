/**
 * @file leader.hpp
 * @author Darshit Desai (darshit@umd.edu)
 * @brief Class for the leader node
 * @version 0.1
 * @date 2023-12-19
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include <memory>
#include <shape_shifters/robot.hpp>
#include <vector>
/**
 * @brief Class for the leader node
 *
 */
class Leader : public rclcpp::Node {
 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  std::vector<std::shared_ptr<robot>> robot_array;

 public:
  /**
   * @brief Construct a new Leader object
   *
   * @param robot_array
   */
  explicit Leader(std::vector<std::shared_ptr<robot>> const &robot_array,
                  bool run_formation_generator);
  /**
   * @brief Callback function for the timer
   *
   */
  void binding_callback();
  /**
   * @brief Method to generate the formation
   *
   */
  void formation_generator();
  /**
   * @brief Method to assign the target position to the robots
   *
   * @param robot_position
   * @param formation_points
   */
  void coordinate_assignment(std::vector<std::vector<double>> robot_position,
                             std::vector<std::vector<double>> formation_points);
  /**
   * @brief
   *
   * @return std::vector<std::vector<double>>
   */
  std::vector<std::vector<double>> import_txt(std::string path);
};
