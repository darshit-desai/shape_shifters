/**
 * @file main.cpp
 * @author  Phase 1 - Shivam Sehgal (ssehgal7@umd.edu) - Driver,
 *                    Patrik Pordi (ppordi@umd.edu) - Navigator,
 *                    Darshit Desai (darshit@umd.edu) - Code designer
 *          Phase 2 - Shivam Sehgal (ssehgal7@umd.edu) - Code designer,
 *                    Patrik Pordi (ppordi@umd.edu) - Driver,
 *                    Darshit Desai (darshit@umd.edu) - Navigator
 * @brief Implementation of main.cpp file
 * @version 0.1
 * @date 2023-12-19
 *
 * @copyright Copyright (c) 2023 Darshit Desai, Patrik Pordi, Shivam Sehgal
 * This code is licensed under the MIT License. Please see the
 * accompanying LICENSE file for the full text of the license.
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "../include/shape_shifters/leader.hpp"
#include "../include/shape_shifters/robot.hpp"
#include "../include/shape_shifters/shapeshift.hpp"
/**
 * @brief Main function for the shape shifters package
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // Make a Multi threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  // Create a vector of shared pointers to Robot objects
  std::vector<std::shared_ptr<robot>> robot_array;
  // Create 24 robots and push them into the vector with its name space and node
  // name
  for (int i = 0; i < 24; i++) {
    std::string name_space = "turtlebot3_" + std::to_string(i);
    std::string node_name = "robot_" + std::to_string(i) + "_node";
    auto node = std::make_shared<robot>(name_space, node_name);
    executor.add_node(node);
    robot_array.push_back(node);
  }
  // Create a leader node and add it to the executor
  auto leader_node = std::make_shared<Leader>(robot_array, true);
  executor.add_node(leader_node);
  // Spin the executor
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
