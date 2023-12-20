/**
 * @file shapeshift.hpp
 *
 * @author  Phase 1 - Shivam Sehgal (ssehgal7@umd.edu) - Driver,
 *                    Patrik Pordi (ppordi@umd.edu) - Navigator,
 *                    Darshit Desai (darshit@umd.edu) - Code designer
 *          Phase 2 - Shivam Sehgal (ssehgal7@umd.edu) - Code designer,
 *                    Patrik Pordi (ppordi@umd.edu) - Driver,
 *                    Darshit Desai (darshit@umd.edu) - Navigator
 * @brief Implementation of Shapeshifters class
 * @version 0.1
 * @date 2023-12-19
 *
 *
 *
 * @copyright Copyright (c) 2023 Darshit Desai, Patrik Pordi, Shivam Sehgal
 * This code is licensed under the MIT License. Please see the
 * accompanying LICENSE file for the full text of the license.
 *
 */

#include "../include/shape_shifters/shapeshift.hpp"

#include <stdexcept>
using std::vector;
/**
 * @brief Construct a new Shapeshift object and initialize the variables
 *
 */
Shapeshifters::Shapeshifters() {
  center_x = 0;
  center_y = 0;
  radius = 10.0;
  side = 9.6;
  circumRadius = 9.6;
  numRobots = 24;
}
/**
 * @brief Set the center of the trajectory
 *
 * @param x  X coordinate of the center
 * @param y  Y coordinate of the center
 */
void Shapeshifters::setCenter(double x, double y) {
  center_x = x;
  center_y = y;
}
/**
 * @brief Set the number of robots in the formation
 *
 * @param n  Number of robots
 */
void Shapeshifters::setNumRobots(double n) { numRobots = n; }
/**
 * @brief Get the number of robots in the formation
 *
 * @return int Number of robots
 */
int Shapeshifters::getNumRobots() { return numRobots; }
/**
 * @brief Method to generate the circle formation
 *
 * @return std::vector<std::vector<double>> 2D vector containing the coordinates
 * of the robots
 */
vector<vector<double>> Shapeshifters::shapeCircle() {
  std::vector<std::vector<double>> shapeCircle_vector;
  // Check for integer robots or negative robots
  if (numRobots <= 0) {
    throw std::invalid_argument("Number of robots must be a positive integer");
  }
  // Check for integer number of robots
  if (numRobots != static_cast<int>(numRobots)) {
    throw std::invalid_argument("Number of robots must be a positive integer");
  }
  for (int i = 0; i < numRobots; i++) {
    // shapeCircle_vector[i][0] =
    //     (center_x + radius * cos(2 * M_PI * i / numRobots));
    // shapeCircle_vector[i][1] =
    //     (center_y + radius * sin(2 * M_PI * i / numRobots));
    shapeCircle_vector.push_back(
        {(center_x + radius * cos(2 * M_PI * i / numRobots)),
         (center_y + radius * sin(2 * M_PI * i / numRobots))});
  }
  return shapeCircle_vector;
}
/**
 * @brief Method to generate the square formation
 *
 * @return std::vector<std::vector<double>> 2D vector containing the coordinates
 * of the robots
 */
vector<vector<double>> Shapeshifters::shapeSquare() {
  // Check for integer robots or negative robots
  if (numRobots <= 0) {
    throw std::invalid_argument("Number of robots must be a positive integer");
  }
  // Check for integer number of robots
  if (numRobots != static_cast<int>(numRobots)) {
    throw std::invalid_argument("Number of robots must be a positive integer");
  }
  std::vector<std::vector<double>> shapeSquare_vector;
  double halfSide = side / 2;
  double step = side / (numRobots / 4);

  for (int i = 0; i < numRobots; i++) {
    if (i < numRobots / 4) {
      shapeSquare_vector.push_back(
          {center_x - halfSide + i * step, center_y - halfSide});
    } else if (i < numRobots / 2) {
      shapeSquare_vector.push_back(
          {center_x + halfSide,
           center_y - halfSide + (i - numRobots / 4) * step});
    } else if (i < 3 * numRobots / 4) {
      shapeSquare_vector.push_back(
          {center_x + halfSide - (i - numRobots / 2) * step,
           center_y + halfSide});
    } else {
      shapeSquare_vector.push_back(
          {center_x - halfSide,
           center_y + halfSide - (i - 3 * numRobots / 4) * step});
    }
  }

  return shapeSquare_vector;
}
/**
 * @brief Method to generate the triangle formation
 *
 * @return std::vector<std::vector<double>> 2D vector containing the coordinates
 * of the robots
 */
vector<vector<double>> Shapeshifters::shapeTriangle() {
  // Check for no. of robots in multiples of 3
  if (static_cast<int>(numRobots) % 3 != 0) {
    throw std::invalid_argument("Number of robots must be a multiple of 3");
  }
  // Check for integer robots or negative robots
  if (numRobots <= 0) {
    throw std::invalid_argument("Number of robots must be a positive integer");
  }
  // Check for integer number of robots
  if (numRobots != static_cast<int>(numRobots)) {
    throw std::invalid_argument("Number of robots must be a positive integer");
  }
  std::vector<std::vector<double>> shapeTriangle_vector;
  double x = center_x - circumRadius;
  double y = center_y + (3 * circumRadius / 2 - circumRadius);
  double stepX = circumRadius / (numRobots / 3);
  double stepY = circumRadius / (numRobots / 3);

  for (int i = 0; i < numRobots; i++) {
    if (i < numRobots / 3) {
      x += stepX;
      y -= stepY;
    } else if (i < 2 * numRobots / 3) {
      x += stepX;
      y += stepY;
    } else {
      x -= 2 * stepX;
    }
    shapeTriangle_vector.push_back({x, y});
  }

  return shapeTriangle_vector;
}
/**
 * @brief Set the radius of the circle trajectory
 *
 * @param r  Radius of the circle
 */
void Shapeshifters::setRadius(double r) {
  if (r <= 0) {
    throw std::invalid_argument("Radius must be greater than 0");
  }
  radius = r;
}
/**
 * @brief Set the circumradius of the triangle trajectory
 *
 * @param r  Circumradius of the triangle
 */
void Shapeshifters::setCircumRadius(double r) {
  if (r <= 0) {
    throw std::invalid_argument("Circumradius must be greater than 0");
  }
  circumRadius = r;
}

