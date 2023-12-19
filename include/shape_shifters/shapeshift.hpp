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
  double center_x;         // X coordinate of the center
  double center_y;         // Y coordinate of the center
  double radius;           // Radius of the circle trajectory
  double numberOfRobots;   // Number of robots in the formation
  double circumRadius;     // Circumradius of the triangle trajectory

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
  void setnumberOfRobots(double n);

  /**
   * @brief Get the number of robots in the formation
   *
   * @return int Number of robots
   */
  int getnumberOfRobots();

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
};

#endif  // INCLUDE_SHAPESHIFT_HPP_
