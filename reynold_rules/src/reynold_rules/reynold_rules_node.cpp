#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "reynold_rules/reynold_rules_node.hpp"
#include "reynold_rules_interfaces/msg/vector_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <iostream>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace reynold_rules
{

using Vector3d = reynold_rules_interfaces::msg::VectorArray;

ReynoldRulesNode::ReynoldRulesNode()
: Node("publisher_node")
{
  for (int n = 1; n <= NUMBER_DRONES; n++) {
    std::string topic_name = "cf_" + std::to_string(n) + "/odom";
    RCLCPP_INFO(get_logger(), "Drone names: %s", topic_name.c_str());
    drones_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        topic_name, 
        10,
        std::bind(&ReynoldRulesNode::odom_callback, this, std::placeholders::_1)
    );
  }

  timer_ = create_wall_timer(
    500ms, std::bind(&ReynoldRulesNode::control_cycle, this));
}

void ReynoldRulesNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr data)
{
  std::string number = data->child_frame_id.substr(std::strlen("cf_"));
  RCLCPP_INFO(get_logger(), "Drone number: %s", number.c_str());
}


void
ReynoldRulesNode::control_cycle()
{
  while (true){
    void();
  }
}

Vector3d
ReynoldRulesNode::separation_rule()
{
  while (true){
    void();
  }
}

Vector3d
ReynoldRulesNode::aligment_rule()
{
  while (true){
    void();
  }
}

Vector3d
ReynoldRulesNode::cohesion_rule()
{
  while (true){
    void();
  }
}

Vector3d
ReynoldRulesNode::nav_2_point_rule()
{
  while (true){
    void();
  }
}

Vector3d
ReynoldRulesNode::avoidance_rule()
{
  while (true){
    void();
  }
}


}  //  namespace reynold_rules
