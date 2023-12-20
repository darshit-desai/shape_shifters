/**
 * @file leader.cpp
 * @author Darshit Desai (darshit@umd.edu)
 * @brief Implementation of the Leader class
 * @version 0.1
 * @date 2023-12-19
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "../include/shape_shifters/leader.hpp"

#include <fstream>
#include <shape_shifters/robot.hpp>
#include <shape_shifters/shapeshift.hpp>
/**
 * @brief Construct a new Leader object
 *
 * @param robot_array
 */
Leader::Leader(std::vector<std::shared_ptr<robot>> const &robot_array,
               bool run_formation_generator)
    : Node("leader_node"), robot_array(robot_array) {
  RCLCPP_INFO_STREAM(rclcpp::get_logger("tbot_walker"),
                     "Setting up Leader Node...");
  auto bind_callback = std::bind(&Leader::binding_callback, this);
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100), bind_callback);
  if (run_formation_generator) {
    formation_generator();
  }
}
/**
 * @brief Callback function for the timer
 *
 */
void Leader::binding_callback() {}
/**
 * @brief Method to generate the formation
 *
 */
void Leader::formation_generator() {
  std::vector<std::vector<double>> formation_points;
  int trajectory_option{0};
  std::cout << "Enter the trajectory option: 1 for circle, 2 for square 3 for "
               "triangle \n ";
  std::cin >> trajectory_option;
  std::vector<std::vector<double>> robot_position;
  // robot_position import from txt file
  std::string path =
      "install/shape_shifters/share/shape_shifters/initial_positions.txt";
  robot_position = import_txt(path);
  formation_points = formation_switch(trajectory_option);
  coordinate_assignment(robot_position, formation_points);
}
/**
 * @brief Make a function for switch case which returns formation points
 * 
 */
std::vector<std::vector<double>> Leader::formation_switch(int trajectory_option) {
  Shapeshifters shapeShift;
  std::vector<std::vector<double>> formation_points;
  shapeShift.setCenter(7.5, 7.5);
  shapeShift.setNumRobots(24);
  switch (trajectory_option) {
    case 1:
      formation_points = shapeShift.shapeCircle();
      std::cout << "Executing circle trajectory \n" << std::endl;
      break;
    case 2:
      formation_points = shapeShift.shapeSquare();
      std::cout << "Executing square trajectory \n" << std::endl;
      break;
    case 3:
      formation_points = shapeShift.shapeTriangle();
      std::cout << "Executing triangle trajectory \n" << std::endl;
      break;
    default:
      std::cout << "Invalid option. Exiting... \n";
      // Return an assert throw
      throw std::invalid_argument("Invalid option. Exiting... \n");
  }
  return formation_points;
}

/**
 * @brief Method to assign the target position to the robots
 *
 * @param robot_position
 * @param formation_points
 */
void Leader::coordinate_assignment(
    std::vector<std::vector<double>> robot_position,
    std::vector<std::vector<double>> formation_points) {
  int target_idx = 0;
  // loop through all the robots and find the closest formation point
  for (size_t i = 0; i < robot_array.size(); i++) {
    // Declare a variable to initialze the minimum distance with extreme of
    // float
    double min_distance = std::numeric_limits<double>::max();
    // get current robot position
    // loop through all the formation_points and find the lowest distance
    // between the robot and the formation point
    for (size_t j = 0; j < formation_points.size(); j++) {
      // calculate the distance between the robot and the formation point
      double distance =
          sqrt(pow(robot_position[i][0] - formation_points[j][0], 2) +
               pow(robot_position[i][1] - formation_points[j][1], 2));
      // if the distance is less than the current distance, update the
      // target point
      if (distance < min_distance) {
        target_idx = j;
        min_distance = distance;
      }
    }
    robot_array[i]->setTarget(formation_points[target_idx][0],
                              formation_points[target_idx][1]);
    // pop the target_idx out of formation_points
    formation_points.erase(formation_points.begin() + target_idx);
  }
}

/**
 * @brief Method to import the txt file
 *
 * @return std::vector<std::vector<double>>
 */
std::vector<std::vector<double>> Leader::import_txt(std::string path) {
  // import the txt file and store the data in a vector
  std::vector<std::vector<double>> robot_position;
  std::ifstream file;
  file.open(path);
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open file: " + path);
  }
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::vector<double> robot_position_line;
    double value;
    while (iss >> value) {
      robot_position_line.push_back(value);
    }
    robot_position.push_back(robot_position_line);
  }
  file.close();
  return robot_position;
}