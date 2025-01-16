#include <vector>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int32.hpp"
#include "reynold_rules/reynold_rules_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace reynold_rules
{

ReynoldRulesNode::ReynoldRulesNode()
: Node("reynold_rules_node")
{
  RCLCPP_INFO(get_logger(), "Initiating reynold rules node...");

  // Make sure the array of the odom for the drones is of the correct size
  if (robots_.size() <= NUMBER_DRONES) {
      robots_.resize(NUMBER_DRONES);  // Redimensiona si es necesario
  }

  for (int n = 1; n <= NUMBER_DRONES; n++) {
    std::string odom_topic = "/cf_" + std::to_string(n) + "/odom";
    std::string vel_topic = "/cf_" + std::to_string(n) + "/cmd_vel";

    publishers_.push_back(create_publisher<geometry_msgs::msg::Twist>(vel_topic, 10));
    subscribers_.push_back(create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&ReynoldRulesNode::odom_callback, this, std::placeholders::_1)
    ));
  }

  // map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
  //    "/map", 
  //    10,
  //    std::bind(&ReynoldRulesNode::map_callback, this, std::placeholders::_1)
  //);

  timer_ = create_wall_timer(
    500ms, std::bind(&ReynoldRulesNode::control_cycle, this));
}

void
ReynoldRulesNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr data)
{
    std::string number = data->child_frame_id.substr(std::strlen("cf_"));

    // Asignación de SharedPtr en el vector
    int drone_number = std::stoi(number);

    robots_[drone_number - 1] = data;
}

void
ReynoldRulesNode::control_cycle()
{
  return;
}

std::vector<geometry_msgs::msg::Vector3>
ReynoldRulesNode::separation_rule()
{
  while (true){
    continue;
  }
}

std::vector<geometry_msgs::msg::Vector3>
ReynoldRulesNode::aligment_rule()
{
  while (true){
    continue;
  }
}

std::vector<geometry_msgs::msg::Vector3>
ReynoldRulesNode::cohesion_rule()
{
  while (true){
    continue;
  }
}

std::vector<geometry_msgs::msg::Vector3>
ReynoldRulesNode::nav_2_point_rule()
{
  while (true){
    continue;
  }
}

std::vector<geometry_msgs::msg::Vector3>
ReynoldRulesNode::avoidance_rule()
{
  while (true){
    continue;
  }
}

}  //  namespace reynold_rules
