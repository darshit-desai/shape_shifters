/**
 * @file shapeshift.hpp
 * @author Darshit Desai (darshit@umd.edu)
 * @brief ShapeShifters class header file
 * @version 0.1
 * @date 2023-12-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef INCLUDE_SHAPESHIFT_HPP_
#define INCLUDE_SHAPESHIFT_HPP_

#include <cmath>
#include <iostream>
#include <vector>

/**
 * @brief Class for defining robot trajectories and shapes
 *
 */
class Shapeshifters {
 private:
  double center_x;      // X coordinate of the center
  double center_y;      // Y coordinate of the center
  double radius;        // Radius of the circle trajectory
  double numRobots;     // Number of robots in the formation
  double circumRadius;  // Circumradius of the triangle trajectory
  double side;          // Side length of the square trajectory
 public:
  /**
   * @brief Construct a new Shapeshift object and initialize the variables
   *
   */
  Shapeshifters();

  /**
   * @brief Set the center of the trajectory
   *
   * @param x  X coordinate of the center
   * @param y  Y coordinate of the center
   */
  void setCenter(double x, double y);

  /**
   * @brief Set the number of robots in the formation
   *
   * @param n  Number of robots
   */
  void setNumRobots(double n);

  /**
   * @brief Get the number of robots in the formation
   *
   * @return int Number of robots
   */
  int getNumRobots();

  /**
   * @brief Function to calculate the circle-shaped trajectory
   *
   * @return std::vector<std::vector<double>> Vector of vectors containing the
   * x and y coordinates of the robots
   */
  std::vector<std::vector<double>> shapeCircle();

  /**
   * @brief Function to calculate the square-shaped trajectory
   *
   * @return std::vector<std::vector<double>> Vector of vectors containing the
   * x and y coordinates of the robots
   */
  std::vector<std::vector<double>> shapeSquare();

  /**
   * @brief Function to calculate the triangle-shaped trajectory
   *
   * @return std::vector<std::vector<double>> Vector of vectors containing the
   * x and y coordinates of the robots
   */
  std::vector<std::vector<double>> shapeTriangle();
  /**
   * @brief Set the radius of the circle trajectory
   *
   * @param r  Radius of the circle
   */
  void setRadius(double r);
  /**
   * @brief Set the circumradius of the triangle trajectory
   *
   * @param r  Circumradius of the triangle
   */
  void setCircumRadius(double r);
};

#endif  // INCLUDE_SHAPESHIFT_HPP_
