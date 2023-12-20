/**
 * @file unit_test.cpp
 * @author Darshit Desai (darshit@umd.edu)
 * @brief Unit tests for the shape shifters package
 * @version 0.1
 * @date 2023-12-19
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <gtest/gtest.h>

#include <memory>
#include <utility>
#include <vector>

#include "../include/shape_shifters/shapeshift.hpp"

/**
 * @brief Construct a new TEST object for the circle formation
 *
 */
TEST(ShapeShiftersTest, circleFormation) {
  Shapeshifters shapeShift;
  std::vector<std::vector<double>> formation_points;
  shapeShift.setCenter(7.5, 7.5);
  shapeShift.setRadius(10.0);
  shapeShift.setNumRobots(24);
  formation_points = shapeShift.shapeCircle();
  // Check the number of points generated
  EXPECT_EQ(formation_points.size(), shapeShift.getNumRobots());
  // Check whether the points lie on the circle circumference
  for (size_t i = 0; i < static_cast<size_t>(formation_points.size()); i++) {
    EXPECT_NEAR(formation_points[i][0] - 7.5, 10 * cos(2 * M_PI * i / 24), 0.1);
    EXPECT_NEAR(formation_points[i][1] - 7.5, 10 * sin(2 * M_PI * i / 24), 0.1);
  }
  // Test for invalid number of robots
  shapeShift.setNumRobots(-10);
  EXPECT_THROW(shapeShift.shapeCircle(), std::invalid_argument);
  // Test for invalid number of robots
  shapeShift.setNumRobots(10.5);
  EXPECT_THROW(shapeShift.shapeCircle(), std::invalid_argument);
}
/**
 * @brief Construct a new TEST object for the negative circle formation for
 * testing the exception
 *
 */
TEST(ShapeShiftersTest, negativeCircleFormation) {
  Shapeshifters shapeShift;
  std::vector<std::vector<double>> formation_points;
  shapeShift.setCenter(7.5, 7.5);
  shapeShift.setNumRobots(24);
  EXPECT_THROW(shapeShift.setRadius(-10.0), std::invalid_argument);
}

/**
 * @brief Construct a new TEST object for the triangle formation
 *
 */
TEST(ShapeShiftersTest, triangleFormation) {
  Shapeshifters shapeShift;
  std::vector<std::vector<double>> formation_points;
  shapeShift.setCenter(7.5, 7.5);
  shapeShift.setNumRobots(24);
  formation_points = shapeShift.shapeTriangle();
  // Check the number of points generated
  EXPECT_EQ(formation_points.size(), 24);
  // Test for invalid number of robots
  shapeShift.setNumRobots(-12);
  EXPECT_THROW(shapeShift.shapeTriangle(), std::invalid_argument);
  // Test for invalid number of robots
  shapeShift.setNumRobots(15.5);
  EXPECT_THROW(shapeShift.shapeTriangle(), std::invalid_argument);
}

/**
 * @brief Construct a new TEST object for the triangle formation
 *
 */
TEST(ShapeshiftersTest, ShapeTriangleInvalidRobotsTest) {
  Shapeshifters shapeshifters;
  double validCircumRadius = 5.0;
  int invalidNumRobots = 11;  // Not a multiple of 3
  shapeshifters.setCircumRadius(validCircumRadius);
  shapeshifters.setNumRobots(invalidNumRobots);
  // Check that the function throws an invalid_argument exception for an invalid
  // number of robots which is checked in shapeTriangle methd
  EXPECT_THROW(shapeshifters.shapeTriangle(), std::invalid_argument);
}
/**
 * @brief Construct a new TEST object for the square formation
 *
 */
TEST(ShapeShiftersTest, squareFormation) {
  Shapeshifters shapeShift;
  std::vector<std::vector<double>> formation_points;
  shapeShift.setCenter(7.5, 7.5);
  shapeShift.setNumRobots(24);
  formation_points = shapeShift.shapeSquare();
  // Check the number of points generated
  EXPECT_EQ(formation_points.size(), 24);
}
/**
 * @brief Construct a new TEST object for the square formation with negative
 * circum radius
 *
 */
TEST(ShapeShiftersTest, negativeSquareFormation) {
  Shapeshifters shapeShift;
  std::vector<std::vector<double>> formation_points;
  shapeShift.setCenter(7.5, 7.5);
  shapeShift.setNumRobots(24);
  EXPECT_THROW(shapeShift.setCircumRadius(-10.0), std::invalid_argument);
  // Test for invalid number of robots
  shapeShift.setNumRobots(-10);
  EXPECT_THROW(shapeShift.shapeSquare(), std::invalid_argument);
}
