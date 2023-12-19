#include "../include/leader.hpp"
#include "../include/shapeshift.hpp"

using std::cin;
using std::cout;
using std::endl;
using std::vector;
using std::placeholders::_1;
using namespace std::chrono_literals;

using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;
using RCL_NODE_PTR = std::shared_ptr<rclcpp::Node>;

Leader::Leader(std::vector<std::shared_ptr<Robot>> const &robot_array)
    : Node("master_node") {
  this->robot_array = robot_array;
  auto processCallback = std::bind(&Leader::process_callback, this);
  this->timer_ = this->create_wall_timer(100ms, processCallback);
  this->traj();
}

void Leader::process_callback() {}
