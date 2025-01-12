#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "reynold_rules/reynold_rules_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<reynold_rules::ReynoldRulesNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}