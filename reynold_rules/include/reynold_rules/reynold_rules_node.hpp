#ifndef REYNOLD_RULES__REYNOLDRULESNODE_HPP_
#define REYNOLD_RULES__REYNOLDRULESNODE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace reynold_rules
{

class ReynoldRulesNode : public rclcpp::Node
{
public:  
  RCLCPP_SMART_PTR_DEFINITIONS(ReynoldRulesNode)
  
  ReynoldRulesNode();
  void control_cycle();

  std::vector<float> separation_rule();
  std::vector<float> aligment_rule();
  std::vector<float> cohesion_rule();
  std::vector<float> nav_2_point_rule();
  std::vector<float> avoidance_rule();

private:

  rclcpp::TimerBase::SharedPtr timer_;
};

}  //  namespace reynold_rules

#endif  // REYNOLD_RULES__REYNOLDRULESNODE_HPP_
