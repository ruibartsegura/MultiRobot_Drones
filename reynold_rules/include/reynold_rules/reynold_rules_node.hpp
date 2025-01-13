#ifndef REYNOLD_RULES__REYNOLDRULESNODE_HPP_
#define REYNOLD_RULES__REYNOLDRULESNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "reynold_rules_interfaces/msg/vector_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <vector>
#include <queue>
#include <set>

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
  static const int NUMBER_DRONES {4};
  std::vector<nav_msgs::msg::Odometry::SharedPtr> drones_;

  // Map
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;

  // Nav_2_Point
  std::vector<geometry_msgs::msg::Point> findPathThroughWaypoints(
      const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& target);

  std::vector<geometry_msgs::msg::Point> findNeighbors(
    const std::vector<geometry_msgs::msg::Point>& waypoints, const geometry_msgs::msg::Point& currentWp, int step = 2);

  bool isPathClear(const std::pair<int, int>& start, const std::pair<int, int>& end);

  geometry_msgs::msg::Point target_point;
  geometry_msgs::msg::Point prev_point;
  std::vector<geometry_msgs::msg::Point> waypoints_;
  std::vector<geometry_msgs::msg::Point> path_;

  // Subscribers
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr data);

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drones_sub1_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drones_sub2_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drones_sub3_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drones_sub4_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drones_sub_;

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr data);
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  

  rclcpp::TimerBase::SharedPtr timer_;
  int a = 3;
};

}  //  namespace reynold_rules

#endif  // REYNOLD_RULES__REYNOLDRULESNODE_HPP_
