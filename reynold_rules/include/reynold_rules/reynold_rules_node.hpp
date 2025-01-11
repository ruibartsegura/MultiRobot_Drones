#ifndef REYNOLD_RULES__REYNOLDRULESNODE_HPP_
#define REYNOLD_RULES__REYNOLDRULESNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace reynold_rules
{

class ReynoldRulesNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ReynoldRulesNode)

  ReynoldRulesNode();
  void control_cycle();

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::Int32 message_;
};

}  //  namespace reynold_rules

#endif  // REYNOLD_RULES__REYNOLDRULESNODE_HPP_
