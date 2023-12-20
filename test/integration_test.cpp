/**
 * @file integration_test.cpp
 * @author Darshit Desai (darshit@umd.edu)
 * @brief Integration tests for the shape shifters package
 * @version 0.1
 * @date 2023-12-20
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <shape_shifters/leader.hpp>
#include <shape_shifters/robot.hpp>
#include <std_msgs/msg/string.hpp>
/**
 * @brief Test suite for the shape shifters package especially the robot class
 *
 */
class TestSuite : public testing::Test {
 protected:
  static void SetUpTestCase() {
    // Initialize ROS before the test suite
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase() {
    // Shutdown ROS after the test suite
    rclcpp::shutdown();
  }

  TestSuite() {}

  ~TestSuite() override {
    // Clear nodes and executor after each test
    robot_node.reset();
    exec.reset();
  }

  std::shared_ptr<robot> robot_node;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec;
};
/**
 * @brief Write a test for the robot constructor and checking the number of
 * publishers
 *
 */
TEST_F(TestSuite, test_single_robot_publishers) {
  // Create and add nodes to the executor before each test
  exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  // Assuming you want to test only one robot (e.g., robot_0)
  std::string r_namespace = "robot_0";
  std::string nodename = "robot_0_controller";
  robot_node = std::make_shared<robot>(r_namespace, nodename);
  exec->add_node(robot_node);
  int pub_count = robot_node->count_publishers("/robot_0/cmd_vel");
  EXPECT_EQ(1, pub_count);  // Update this based on your expected count
}
/**
 * @brief Write a test for the robot constructor and checking the number of
 * subscribers
 *
 */
TEST_F(TestSuite, slave_spawn_testing_subscribers) {
  int nodes = 10;
  int pub_count = 0;
  rclcpp::executors::MultiThreadedExecutor exec;
  std::vector<std::shared_ptr<robot>> robot_array;
  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto nodename = "robot_" + std::to_string(i) + "_controller";
    auto robot_node = std::make_shared<robot>(r_namespace, nodename);
    exec.add_node(robot_node);
    robot_array.push_back(robot_node);
  }
  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto number_of_subs =
        robot_array[i]->count_subscribers("/" + r_namespace + "/odom");
    pub_count = pub_count + static_cast<int>(number_of_subs);
  }
  EXPECT_EQ(nodes, pub_count);
}

/**
 * @brief Write a test for set target function in robot class
 *
 */
TEST_F(TestSuite, set_target_test) {
  std::string r_namespace = "robot_0";
  std::string nodename = "robot_0_controller";
  robot_node = std::make_shared<robot>(r_namespace, nodename);
  robot_node->setTarget(10, 10);
  // Check if the target is set correctly using the getter function
  EXPECT_EQ(robot_node->getTarget().first, 10);
  EXPECT_EQ(robot_node->getTarget().second, 10);
}
/**
 * @brief Write a test for calculate velocity function in robot class and other
 * small tests
 *
 */
TEST_F(TestSuite, calculate_velocity_test) {
  std::string r_namespace = "robot_0";
  std::string nodename = "robot_0_controller";
  robot_node = std::make_shared<robot>(r_namespace, nodename);
  robot_node->setTarget(10, 10);
  robot_node->setCurrentPosition(10, 10);
  robot_node->setHeadingAngle(0.0);
  // Check if the velocity is calculated correctly
  EXPECT_EQ(robot_node->velocity_calculation().first, 0);
  EXPECT_EQ(robot_node->velocity_calculation().second, 0);
  // Write a test for setting and getting heading angle
  robot_node->setHeadingAngle(1.0);
  EXPECT_EQ(robot_node->getHeadingAngle(), 1.0);
  // Write a test for calculating the distance to the target
  EXPECT_EQ(robot_node->get_goal_distance(), 0);
}
/**
 * @brief Write a test for the odom callback function in robot class
 *
 */
TEST_F(TestSuite, odom_callback_test) {
  std::string r_namespace = "robot_0";
  std::string nodename = "robot_0_controller";
  robot_node = std::make_shared<robot>(r_namespace, nodename);

  // Create an Odometry message
  auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  odom_msg->pose.pose.position.x = 1.0;
  odom_msg->pose.pose.position.y = 2.0;
  odom_msg->pose.pose.orientation.x = 0.0;
  odom_msg->pose.pose.orientation.y = 0.0;
  odom_msg->pose.pose.orientation.z = 0.0;
  odom_msg->pose.pose.orientation.w = 1.0;

  // Call the odom_callback function
  robot_node->odom_callback(odom_msg);

  // Check if the current position and heading angle are updated correctly
  EXPECT_EQ(robot_node->getCurrentPosition().first, 1.0);
  EXPECT_EQ(robot_node->getCurrentPosition().second, 2.0);
  EXPECT_EQ(robot_node->getHeadingAngle(), 0.0);
}
/**
 * @brief Write a test for the velocity callback function in robot class
 *
 */
TEST_F(TestSuite, velocity_callback_test) {
  std::string r_namespace = "robot_0";
  std::string nodename = "robot_0_controller";
  robot_node = std::make_shared<robot>(r_namespace, nodename);

  // Set a target for the robot
  robot_node->setTarget(10, 10);

  // Set the current position of the robot
  robot_node->setCurrentPosition(10, 10);

  // Set the heading angle of the robot
  robot_node->setHeadingAngle(0.0);

  // Create a publisher for cmd_vel topic
  auto publisher = robot_node->create_publisher<geometry_msgs::msg::Twist>(
      "/" + r_namespace + "/cmd_vel", 10);

  // Subscribe to cmd_vel topic to get published messages
  auto subscription =
      robot_node->create_subscription<geometry_msgs::msg::Twist>(
          "/" + r_namespace + "/cmd_vel", 10,
          [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            // Check the published message values
            EXPECT_EQ(msg->linear.x, 0);
            EXPECT_EQ(msg->angular.z, 0);
          });

  // Wait for a short time to allow the subscription to be set up
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Call the velocity_callback function
  robot_node->velocity_callback();

  // Spin the node to process callbacks
  rclcpp::spin_some(robot_node);

  // Wait for a short time to allow the subscription callback to be executed
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
/**
 * @brief Leader class test suite
 *
 */
class LeaderTest : public testing::Test {
 protected:
  static void SetUpTestCase() {
    // Initialize ROS before the test suite
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase() {
    // Shutdown ROS after the test suite
    rclcpp::shutdown();
  }

  LeaderTest() {}

  ~LeaderTest() override {
    // Clear nodes and executor after each test
    leader_node.reset();
    exec.reset();
  }

  std::shared_ptr<Leader> leader_node;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec;
};
/**
 * @brief Write a test for the leader constructor
 *
 */
TEST_F(LeaderTest, leader_constructor_test) {
  // Create and add nodes to the executor before each test
  exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // Assuming you have a vector of robot nodes (replace with your actual robot
  // nodes)
  std::vector<std::shared_ptr<robot>> robot_array;

  // Create Leader node
  leader_node = std::make_shared<Leader>(robot_array, false);
  exec->add_node(leader_node);

  // Check if the Leader node is not null
  ASSERT_NE(leader_node, nullptr);
}
/**
 * @brief Write a test for the import_txt function in leader class
 *
 */
TEST_F(LeaderTest, import_txt_test) {
  // Create Leader instance (assuming you have a vector of robot nodes)
  std::vector<std::shared_ptr<robot>> robot_array;
  leader_node = std::make_shared<Leader>(robot_array, false);

  // Generate a unique temporary test file path using mkstemp
  char tempFileName[] = "/tmp/mytempfile_XXXXXX";
  int fd = mkstemp(tempFileName);
  close(fd);  // Close the file immediately; we don't need it for this test
  std::string testFilePath = tempFileName;

  // Open the file for writing
  std::ofstream tempFile(testFilePath);
  // Write dummy coordinates to the file
  tempFile
      << "0.0 10.0\n1.0 2.0\n";  // Assuming two sets of coordinates in the file
  tempFile.close();

  // Call the import_txt function
  std::vector<std::vector<double>> importedCoordinates =
      leader_node->import_txt(testFilePath);

  // Verify that the imported coordinates match the expected values
  ASSERT_EQ(importedCoordinates.size(),
            2);  // Assuming two sets of coordinates in the file
  ASSERT_EQ(importedCoordinates[0].size(), 2);
  ASSERT_EQ(importedCoordinates[0][0], 0.0);
  ASSERT_EQ(importedCoordinates[0][1], 10.0);
  ASSERT_EQ(importedCoordinates[1].size(), 2);
  ASSERT_EQ(importedCoordinates[1][0], 1.0);  // Dummy coordinate
  ASSERT_EQ(importedCoordinates[1][1], 2.0);  // Dummy coordinate

  // Clean up: The temporary test file will be automatically deleted when the
  // program exits
}
/**
 * @brief Write a test for the import_txt function in leader class checking for
 * invalid path
 *
 */
TEST_F(LeaderTest, import_txt_invalid_path_test) {
  // Create Leader instance (assuming you have a vector of robot nodes)
  std::vector<std::shared_ptr<robot>> robot_array;
  leader_node = std::make_shared<Leader>(robot_array, false);

  // Attempt to import from an invalid file path
  std::string invalidFilePath = "nonexistent_file.txt";
  ASSERT_THROW(leader_node->import_txt(invalidFilePath), std::runtime_error);
}
/**
 * @brief Write a test for the coordinate_assignment method in Leader class
 *
 */
TEST_F(LeaderTest, coordinate_assignment_test) {
  // Create and add nodes to the executor before the test
  exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // Assuming you have a vector of robot nodes (replace with your actual robot
  // nodes)
  std::vector<std::shared_ptr<robot>> robot_array;

  // Create Leader node
  leader_node = std::make_shared<Leader>(robot_array, false);
  exec->add_node(leader_node);

  // Assuming you have valid robot positions and formation points
  std::vector<std::vector<double>> robot_positions = {
      {0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}};
  std::vector<std::vector<double>> formation_points = {
      {5.0, 5.0}, {6.0, 6.0}, {7.0, 7.0}};

  // Set the robot positions (you may need to modify based on your actual robot
  // structure)
  for (size_t i = 0; i < robot_array.size() && i < robot_positions.size();
       ++i) {
    robot_array[i]->setCurrentPosition(robot_positions[i][0],
                                       robot_positions[i][1]);
  }

  // Call the coordinate_assignment method
  leader_node->coordinate_assignment(robot_positions, formation_points);

  // Check if the target positions are correctly assigned to the robots
  for (size_t i = 0; i < robot_array.size() && i < formation_points.size();
       ++i) {
    auto target = robot_array[i]->getTarget();
    EXPECT_EQ(target.first, formation_points[i][0]);
    EXPECT_EQ(target.second, formation_points[i][1]);
  }
}
/**
 * @brief Write a test for the formation_switch function in Leader class
 *
 */
TEST_F(LeaderTest, formation_switch_test) {
  // Create Leader instance (assuming you have a vector of robot nodes)
  std::vector<std::shared_ptr<robot>> robot_array;
  leader_node = std::make_shared<Leader>(robot_array, false);

  // Assuming you have valid robot positions and formation points
  std::vector<std::vector<double>> formation_points;

  // Call the formation_switch function for each trajectory option
  formation_points = leader_node->formation_switch(1);
  EXPECT_EQ(formation_points.size(), 24);
  formation_points = leader_node->formation_switch(2);
  EXPECT_EQ(formation_points.size(), 24);
  formation_points = leader_node->formation_switch(3);
  EXPECT_EQ(formation_points.size(), 24);
  // Check for invalid trajectory option
  ASSERT_THROW(leader_node->formation_switch(4), std::invalid_argument);
}
