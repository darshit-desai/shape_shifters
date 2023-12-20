/**
 * @file leader.hpp
 * @author  Phase 1 - Shivam Sehgal (ssehgal7@umd.edu) - Driver,
 *                    Patrik Pordi (ppordi@umd.edu) - Navigator,
 *                    Darshit Desai (darshit@umd.edu) - Code designer
 *          Phase 2 - Shivam Sehgal (ssehgal7@umd.edu) - Code designer,
 *                    Patrik Pordi (ppordi@umd.edu) - Driver,
 *                    Darshit Desai (darshit@umd.edu) - Navigator
 * @brief Leader class
 * @version 0.1
 * @date 2023-12-19
 *
 * @copyright Copyright (c) 2023 Darshit Desai, Patrik Pordi, Shivam Sehgal
 * This code is licensed under the MIT License. Please see the
 * accompanying LICENSE file for the full text of the license.
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
   * @param run_formation_generator
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
   * @param path
   *
   * @return std::vector<std::vector<double>>
   */
  std::vector<std::vector<double>> import_txt(std::string path);
  /**
   * @brief Make a function for switch case which returns formation points
   * @param trajectory_option
   * @return std::vector<std::vector<double>>
   */
  std::vector<std::vector<double>> formation_switch(int trajectory_option);
};
