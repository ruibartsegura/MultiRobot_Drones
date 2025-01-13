#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "reynold_rules/reynold_rules_node.hpp"
#include "reynold_rules_interfaces/msg/vector_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <set>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace reynold_rules
{

using Vector3d = reynold_rules_interfaces::msg::VectorArray;

ReynoldRulesNode::ReynoldRulesNode()
: Node("publisher_node"), map_(nullptr)
{
  // List of waypoint to de nav_2_point
  for (int x = -4; x <= 4; x += 2) {
    for (int y = -4; y <= 4; y += 2) {
        geometry_msgs::msg::Point wp;
        wp.x = x;
        wp.y = y;
        waypoints_.push_back(wp);
        RCLCPP_INFO(get_logger(), "Wp.x: %f", wp.x);
        RCLCPP_INFO(get_logger(), "Wp.y: %f", wp.y);
    }
  }

  for (int n = 1; n <= NUMBER_DRONES; n++) {
    std::string topic_name = "/cf_" + std::to_string(n) + "/odom";
    RCLCPP_INFO(get_logger(), "Drone names: %s", topic_name.c_str());
  }
    drones_sub1_ = create_subscription<nav_msgs::msg::Odometry>(
        "/cf_1/odom", 
        10,
        std::bind(&ReynoldRulesNode::odom_callback, this, std::placeholders::_1)
    );
    drones_sub2_ = create_subscription<nav_msgs::msg::Odometry>(
        "/cf_2/odom", 
        10,
        std::bind(&ReynoldRulesNode::odom_callback, this, std::placeholders::_1)
    );
    drones_sub3_ = create_subscription<nav_msgs::msg::Odometry>(
        "/cf_3/odom", 
        10,
        std::bind(&ReynoldRulesNode::odom_callback, this, std::placeholders::_1)
    );
    drones_sub4_ = create_subscription<nav_msgs::msg::Odometry>(
        "/cf_4/odom", 
        10,
        std::bind(&ReynoldRulesNode::odom_callback, this, std::placeholders::_1)
    );

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 
      10,
      std::bind(&ReynoldRulesNode::map_callback, this, std::placeholders::_1)
  );

  timer_ = create_wall_timer(
    500ms, std::bind(&ReynoldRulesNode::control_cycle, this));
}

void ReynoldRulesNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr data)
{
    std::string number = data->child_frame_id.substr(std::strlen("cf_"));
    RCLCPP_INFO(get_logger(), "Drone number: %s", number.c_str());

    // Asignación de SharedPtr en el vector
    int drone_number = std::stoi(number);

    // Asegúrate de que el vector tiene suficiente espacio
    if (drones_.size() <= drone_number) {
        drones_.resize(drone_number + 1);  // Redimensiona si es necesario
    }

    drones_[drone_number] = data;
}


void ReynoldRulesNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr data)
{
  if (this->map_ == nullptr) {  // Usamos 'this->' para acceder a la variable miembro
    this->map_ = data;  // Asigna el primer mapa recibido a map_
    this->map_->data;   // Usa map_ para acceder a los datos si es necesario
  }
}


void
ReynoldRulesNode::control_cycle()
{
  void();

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
