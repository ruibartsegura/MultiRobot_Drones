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
  declare_parameter<int>("n_robots", 10);
  get_parameter("n_robots", n_robots_);
}

void
ReynoldRulesNode::control_cycle()
{
  while (true){
    void();
  }
}


double
ReynoldRulesNode::get_distance(geometry_msgs::msg::Point pos1, geometry_msgs::msg::Point pos2){
  auto x = pos1.x - pos2.x;
  auto y = pos1.y - pos2.y;
  return sqrt(x * x + y * y);
}

geometry_msgs::msg::Point
ReynoldRulesNode::calc_vector(geometry_msgs::msg::Point position, int num){
  geometry_msgs::msg::Point repulsive_vector; //k is the cte of force
  double k = 0.01;

  for (int i = 0; i < n_robots_; i++){
    // Avoid calculating the robotr's own vector
    if (i != num){
      // Get the distance betwen the robots
      double dist = get_distance(position, robots_[i].pose.pose.position);
    

      // Check if the distance is in the radious
      if ((dist > 0.0) && (dist < view_range_)){
        // Get the x, y coords of the vector
        auto x = position.x - robots_[i].pose.pose.position.x;
        auto y = position.y - robots_[i].pose.pose.position.y;

        // Normalize the vector
        auto norm = sqrt(x * x + y * y);

        double direction[2];
        if (norm > 0){
            direction[0] = x / norm;
            direction[1] = y / norm;
        } else {
            direction[0] = 0.00;
            direction[1] = 0.00;
        }

        // Magnitude od the repulsice vector
        auto magnitude = k / (dist * dist);

        // Sum to the total the repulsive vector of robot_i
        repulsive_vector.x += magnitude * direction[0];
        repulsive_vector.y += magnitude * direction[1];
      }
    }
  }
  return repulsive_vector;
}

Vector3d
ReynoldRulesNode::separation_rule()
{

  declare_parameter<int>("view_range", 0.3);
  get_parameter("view_range", view_range_);
  while (true){
    for (int i = 0; i < n_robots_; i++){
        separation_vectors_.push_back(
            calc_vector(robots_[i].pose.pose.position, i)
        );
    }
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
