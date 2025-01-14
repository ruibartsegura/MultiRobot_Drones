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

  // Make sure the array of the odom for the drones is of the correct size
  if (drones_.size() <= NUMBER_DRONES) {
      drones_.resize(NUMBER_DRONES);  // Redimensiona si es necesario
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
      "/map", 
      10,
      std::bind(&ReynoldRulesNode::map_callback, this, std::placeholders::_1)
  );

  checkPathsBetweenWaypoints();

  timer_ = create_wall_timer(
    500ms, std::bind(&ReynoldRulesNode::control_cycle, this));
}

void ReynoldRulesNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr data)
{
    std::string number = data->child_frame_id.substr(std::strlen("cf_"));
    RCLCPP_INFO(get_logger(), "Drone number: %s", number.c_str());

    // Asignación de SharedPtr en el vector
    int drone_number = std::stoi(number);

    drones_[drone_number] = data;
}

void ReynoldRulesNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr data)
{
  if (this->map_ == nullptr) {  // Usamos 'this->' para acceder a la variable miembro
    this->map_ = data;  // Asigna el primer mapa recibido a map_
    this->map_->data;   // Usa map_ para acceder a los datos si es necesario
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

  for (int i = 0; i < NUMBER_DRONES; i++){
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
    for (int i = 0; i < NUMBER_DRONES; i++){
        separation_vectors_.push_back(
            calc_vector(robots_[i].pose.pose.position, i)
        );
    }
  }
}

Vector3d
ReynoldRulesNode::aligment_rule()
{
  void();
}

Vector3d
ReynoldRulesNode::cohesion_rule()
{
  void();
}

// Find the neighbors of a waypoint
std::vector<geometry_msgs::msg::Point> ReynoldRulesNode::findNeighbors(
  const std::vector<geometry_msgs::msg::Point>& waypoints, 
  const geometry_msgs::msg::Point& currentWp, int step)
{
  std::vector<geometry_msgs::msg::Point> neighbors;
  for (const auto& wp : waypoints) {
    if ((abs(wp.x - currentWp.x) == step && wp.y == currentWp.y) ||
      (abs(wp.y - currentWp.y) == step && wp.x == currentWp.x)) {
      neighbors.push_back(wp);
    }
  }
  return neighbors;
}

// Check if between waypoints there is no obstacle
bool ReynoldRulesNode::isPathClear(const std::pair<int, int>& start, const std::pair<int, int>& end) {
  if (!this->map_) return false;

  int startI = static_cast<int>((start.second - this->map_->info.origin.position.y) / this->map_->info.resolution);
  int startJ = static_cast<int>((start.first - this->map_->info.origin.position.x) / this->map_->info.resolution);
  int endI = static_cast<int>((end.second - this->map_->info.origin.position.y) / this->map_->info.resolution);
  int endJ = static_cast<int>((end.first - this->map_->info.origin.position.x) / this->map_->info.resolution);

  RCLCPP_WARN(this->get_logger(), "startI %d).", startI);
  RCLCPP_WARN(this->get_logger(), "startJ %d).", startJ);
  RCLCPP_WARN(this->get_logger(), "endI %d).", endI);
  RCLCPP_WARN(this->get_logger(), "endJ %d).", endJ);

  int deltaI = abs(endI - startI);
  int deltaJ = abs(endJ - startJ);
  int signI = (endI > startI) ? 1 : -1;
  int signJ = (endJ > startJ) ? 1 : -1;

  int error = deltaI - deltaJ;
  int currentI = startI, currentJ = startJ;

  while (true) {
    int index = this->map_->info.width * currentI + currentJ;
    if (index < 0 || index >= static_cast<int>(this->map_->data.size())) return false;
    if (this->map_->data[index] == 100) return false;

    if (currentI == endI && currentJ == endJ) break;

    int error2 = 2 * error;
    if (error2 > -deltaJ) {
      error -= deltaJ;
      currentI += signI;
    }
    if (error2 < deltaI) {
      error += deltaI;
      currentJ += signJ;
    }
  }

  return true;
}

Vector3d
ReynoldRulesNode::nav_2_point_rule()
{
  void();
}

Vector3d
ReynoldRulesNode::avoidance_rule()
{
  void();
}

void
ReynoldRulesNode::control_cycle()
{
  //std::cout << "Map" << this->map_ << std::endl;
  return;
}

void ReynoldRulesNode::checkPathsBetweenWaypoints() {
    for (const auto& wp : waypoints_) {
        auto neighbors = findNeighbors(waypoints_, wp);
        for (const auto& neighbor : neighbors) {
            auto start = std::make_pair(wp.x, wp.y);
            auto end = std::make_pair(neighbor.x, neighbor.y);
            if (isPathClear(start, end)) {
                RCLCPP_INFO(this->get_logger(), "Free way between (%f, %f) and (%f, %f).",
                            start.first, start.second, end.first, end.second);
            } else {
                RCLCPP_WARN(this->get_logger(), "Obstacle between (%f, %f) and (%f, %f).",
                            start.first, start.second, end.first, end.second);
            }
        }
    }
}

}  //  namespace reynold_rules
