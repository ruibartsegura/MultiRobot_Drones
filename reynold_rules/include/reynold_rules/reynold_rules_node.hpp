#ifndef REYNOLD_RULES_NODE_HPP_
#define REYNOLD_RULES_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace reynold_rules
{

class Reynold_Rules_Node : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Reynold_Rules_Node)

  Reynold_Rules_Node();
  void control_cycle();

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::Int32 message_;
};

}  //  namespace reynold_rules

#endif  // REYNOLD_RULES_NODE_HPP_
