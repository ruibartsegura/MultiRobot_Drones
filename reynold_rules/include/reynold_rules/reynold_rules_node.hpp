#ifndef REYNOLD_RULES__REYNOLDRULESNODE_HPP_
#define REYNOLD_RULES__REYNOLDRULESNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "reynold_rules_interfaces/msg/vector_array.hpp"

namespace reynold_rules
{

class ReynoldRulesNode : public rclcpp::Node
{
public:
  using Vector3d = reynold_rules_interfaces::msg::VectorArray;
  
  RCLCPP_SMART_PTR_DEFINITIONS(ReynoldRulesNode)
  
  ReynoldRulesNode();
  void control_cycle();

  Vector3d separation_rule();
  Vector3d aligment_rule();
  Vector3d cohesion_rule();
  Vector3d nav_2_point_rule();
  Vector3d avoidance_rule();

private:

  rclcpp::TimerBase::SharedPtr timer_;
};

}  //  namespace reynold_rules

#endif  // REYNOLD_RULES__REYNOLDRULESNODE_HPP_
