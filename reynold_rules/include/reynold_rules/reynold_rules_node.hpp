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
  RCLCPP_SMART_PTR_DEFINITIONS(ReynoldRulesNode)
  
  ReynoldRulesNode();
  void control_cycle();

  std::vector<float> separation_rule();
  std::vector<float> aligment_rule();
  std::vector<float> cohesion_rule();
  std::vector<float> nav_2_point_rule();
  std::vector<float> avoidance_rule();

private:
  static const int NUMBER_DRONES {4};
  double MAX_LIN_VEL {0.3};
  double DIST_THRESHOLD {0.3};
  std::vector<nav_msgs::msg::Odometry::SharedPtr> robots_;

  double get_distance(geometry_msgs::msg::Point pos1, geometry_msgs::msg::Point pos2);
  geometry_msgs::msg::Vector3 calc_vector(geometry_msgs::msg::Point position, int num);

  // Map
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;

  // Separation
  int view_range_;

  // Nav_2_Point
  std::vector<geometry_msgs::msg::Point> findPathThroughWaypoints(
      const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& target);

  std::vector<geometry_msgs::msg::Point> findNeighbors(
    const std::vector<geometry_msgs::msg::Point>& waypoints, const geometry_msgs::msg::Point& currentWp, int step = 2);

  bool isPathClear(const std::pair<int, int>& start, const std::pair<int, int>& end);

  geometry_msgs::msg::Vector3 vector_2_points(geometry_msgs::msg::Point point1, geometry_msgs::msg::Point point2);

  geometry_msgs::msg::Point target_point;
  geometry_msgs::msg::Point prev_point;
  std::vector<geometry_msgs::msg::Point> waypoints_;
  std::vector<geometry_msgs::msg::Point> path_;

  // Subscribers
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr data);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drones_sub1_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drones_sub2_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drones_sub3_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drones_sub4_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drones_sub_;

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr data);
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  

  void checkPathsBetweenWaypoints();

  rclcpp::TimerBase::SharedPtr timer_;
};

}  //  namespace reynold_rules

#endif  // REYNOLD_RULES__REYNOLDRULESNODE_HPP_
