#include <vector>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int32.hpp"
#include "reynold_rules/reynold_rules_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace reynold_rules
{

ReynoldRulesNode::ReynoldRulesNode()
: Node("publisher_node")
{
  timer_ = create_wall_timer(
    500ms, std::bind(&ReynoldRulesNode::control_cycle, this));
}

void
ReynoldRulesNode::control_cycle()
{
  while (true){
    continue;
  }
}

std::vector<float>
ReynoldRulesNode::separation_rule()
{
  while (true){
    continue;
  }
}

std::vector<float>
ReynoldRulesNode::aligment_rule()
{
  while (true){
    continue;
  }
}

std::vector<float>
ReynoldRulesNode::cohesion_rule()
{
  while (true){
    continue;
  }
}

std::vector<float>
ReynoldRulesNode::nav_2_point_rule()
{
  while (true){
    continue;
  }
}

std::vector<float>
ReynoldRulesNode::avoidance_rule()
{
  while (true){
    continue;
  }
}

}  //  namespace reynold_rules
