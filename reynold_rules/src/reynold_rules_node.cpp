#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "reynold_rules/reynold_rules_Node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace reynold_rules
{

Reynold_Rules_Node::Reynold_Rules_Node()
: Node("publisher_node")
{
  publisher_ = create_publisher<std_msgs::msg::Int32>("int_topic", 10);
  timer_ = create_wall_timer(
    500ms, std::bind(&Reynold_Rules_Node::control_cycle, this));
}

void
Reynold_Rules_Node::control_cycle()
{
  message_.data += 1;
  publisher_->publish(message_);
}

}  //  namespace reynold_rules
