#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "reynold_rules/reynold_rules_node.hpp"
#include "reynold_rules_interfaces/msg/vector_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace reynold_rules
{

using Vector3d = reynold_rules_interfaces::msg::VectorArray;

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
    int i = 1;
  }
}

Vector3d
ReynoldRulesNode::separation_rule()
{
  while (true){
    int i = 2;
  }
}

Vector3d
ReynoldRulesNode::aligment_rule()
{
  while (true){
    int i = 1;
  }
}

Vector3d
ReynoldRulesNode::cohesion_rule()
{
  while (true){
    int i = 1;
  }
}

Vector3d
ReynoldRulesNode::nav_2_point_rule()
{
  while (true){
    int i = 1;
  }
}

Vector3d
ReynoldRulesNode::avoidance_rule()
{
  while (true){
    int i = 1;
  }
}


}  //  namespace reynold_rules
