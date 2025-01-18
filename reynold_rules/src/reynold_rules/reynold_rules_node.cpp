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

using ArrayVector3d = reynold_rules_interfaces::msg::VectorArray;

double
compute_norm(const std::vector<double>& vec)
{
  double sum_of_squares = std::accumulate(vec.begin(), vec.end(), 0.0,
    [](double sum, double val) { return sum + val * val; }); 
  return sqrt(sum_of_squares);
}

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
    }
  }

  // Make sure the array of the odom for the drones is of the correct size
  if (robots_.size() <= NUMBER_DRONES) {
      robots_.resize(NUMBER_DRONES);  // Redimensiona si es necesario
  }

  for (int n = 1; n <= NUMBER_DRONES; n++) {
    std::string topic_name = "/cf_" + std::to_string(n) + "/odom";
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

  // retrieve parameters
  declare_parameter<int>("view_range", 0.3);
  get_parameter("view_range", view_range_);

  declare_parameter<int>("view_angle", 60);
  get_parameter("view_angle", view_angle_);

  declare_parameter<int>("view_split", 0);
  get_parameter("view_split", view_split_);
}

void ReynoldRulesNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr data)
{
    std::string number = data->child_frame_id.substr(std::strlen("cf_"));

    // Asignación de SharedPtr en el vector
    int drone_number = std::stoi(number);

    robots_[drone_number] = data;
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

geometry_msgs::msg::Vector3
ReynoldRulesNode::calc_sep_vector(geometry_msgs::msg::Point position, int num)
{
  geometry_msgs::msg::Vector3 repulsive_vector; //k is the cte of force
  double k = 0.01;

  for (int i = 0; i < NUMBER_DRONES; i++){
    // Avoid calculating the robotr's own vector
    if (i != num){
      // Get the distance betwen the robots
      double dist = get_distance(position, robots_[i]->pose.pose.position);

      // Check if the distance is in the radious
      if ((dist > 0.0) && (dist < view_range_)){
        // Get the x, y coords of the vector
        auto x = position.x - robots_[i]->pose.pose.position.x;
        auto y = position.y - robots_[i]->pose.pose.position.y;

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

ArrayVector3d
ReynoldRulesNode::separation_rule()
{
  ArrayVector3d separation_vectors;

  for (int i = 0; i < NUMBER_DRONES; i++) {
    geometry_msgs::msg::Vector3 vec = calc_sep_vector(this->robots_[i]->pose.pose.position, i);
    separation_vectors.vectors.push_back(vec); // Agregar al vector del mensaje
  }

  return separation_vectors;
}

geometry_msgs::msg::Vector3
ReynoldRulesNode::calc_average_velocity()
{
  geometry_msgs::msg::Vector3 avg_velocity;

  for (nav_msgs::msg::Odometry::SharedPtr robot : robots_) {
      avg_velocity.x += robot->twist.twist.linear.x;
      avg_velocity.y += robot->twist.twist.linear.y;
  }

  avg_velocity.x /= NUMBER_DRONES;
  avg_velocity.y /= NUMBER_DRONES;

  return avg_velocity;
}

ArrayVector3d
ReynoldRulesNode::aligment_rule()
{
  geometry_msgs::msg::Vector3 avg_velocity = calc_average_velocity();
  ArrayVector3d alignment_vectors;

  for (nav_msgs::msg::Odometry::SharedPtr robot : robots_) {
    geometry_msgs::msg::Vector3 alignment_vector;
    alignment_vector.x = avg_velocity.x - robot->twist.twist.linear.x;
    alignment_vector.y = avg_velocity.y - robot->twist.twist.linear.y;
    alignment_vectors.vectors.push_back(alignment_vector);
  }
  return alignment_vectors;
}

geometry_msgs::msg::Point
ReynoldRulesNode::calc_average_pos(std::vector<nav_msgs::msg::Odometry> positions)
{
  geometry_msgs::msg::Point average_pos;

  for (nav_msgs::msg::Odometry position : positions) {
    average_pos.x += position.pose.pose.position.x;
    average_pos.y += position.pose.pose.position.y;
  }

  average_pos.x = average_pos.x / positions.size();
  average_pos.y = average_pos.y / positions.size();

  return average_pos;

}

geometry_msgs::msg::Vector3
ReynoldRulesNode::calc_cohesion_vector(geometry_msgs::msg::Point robot_pos)
{
  geometry_msgs::msg::Vector3 vector;
  std::vector<nav_msgs::msg::Odometry> neighbors;

  // Make a list with all robots poses in the range of cohesion
  for (nav_msgs::msg::Odometry::SharedPtr robot : robots_) {
      double dist = get_distance(robot_pos, robot->pose.pose.position);

      if (dist < view_range_) {
          neighbors.push_back(*robot);
      }
  }

  // Calculate vector from robot_pos to average_pos of neighbors
  geometry_msgs::msg::Point average_pos = calc_average_pos(neighbors);
  vector.x = average_pos.x - robot_pos.x;
  vector.y = average_pos.y - robot_pos.y;

  return vector;
}

ArrayVector3d
ReynoldRulesNode::cohesion_rule()
{
  ArrayVector3d cohesion_vectors;

  for (nav_msgs::msg::Odometry::SharedPtr robot : robots_) {
      geometry_msgs::msg::Vector3 cohesion_vector = calc_cohesion_vector(robot->pose.pose.position);
      cohesion_vectors.vectors.push_back(cohesion_vector);
  }

  return cohesion_vectors;
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
  int signI;
  int signJ;
  if (endI > startI) {
    signI = 1;
  } else {
    signI = -1;
  }

  if (endJ > startJ) {
    signJ = 1;
  } else {
    signJ = -1;
  }

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

std::vector<geometry_msgs::msg::Point> ReynoldRulesNode::findPathThroughWaypoints(
    const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& target)
{
  using Position2D = std::pair<int, int>;

  Position2D startPos = {static_cast<int>(start.x), static_cast<int>(start.y)};
  Position2D targetPos = {static_cast<int>(target.x), static_cast<int>(target.y)};

  std::queue<std::pair<Position2D, std::vector<Position2D>>> queue;
  std::set<Position2D> visited;
  std::map<Position2D, geometry_msgs::msg::Point> waypointMap;
  std::vector<geometry_msgs::msg::Point> path;

  for (const auto& wp : this->waypoints_) {
    waypointMap[{static_cast<int>(wp.x), static_cast<int>(wp.y)}] = wp;
  }

  queue.push({startPos, {startPos}});
  visited.insert(startPos);

  while (!queue.empty()) {
    auto [currentPos, path] = queue.front();
    queue.pop();

    if (currentPos == targetPos) {
      std::vector<geometry_msgs::msg::Point> result;
      for (const auto& pos : path) {
        result.push_back(waypointMap[pos]);
      }
      return result;
    }

    const auto& currentWp = waypointMap[currentPos];
    auto neighbors = findNeighbors(waypoints_, currentWp);

    for (const auto& neighbor : neighbors) {
      Position2D neighborPos = {static_cast<int>(neighbor.x), static_cast<int>(neighbor.y)};
      if (visited.find(neighborPos) == visited.end() && isPathClear(currentPos, neighborPos)) {
        visited.insert(neighborPos);
        auto newPath = path;
        newPath.push_back(neighborPos);
        queue.push({neighborPos, newPath});
      }
    }
  }

  RCLCPP_WARN(this->get_logger(), "No available route found..");
  return {};
}

geometry_msgs::msg::Vector3
ReynoldRulesNode::vector_2_points(
  const geometry_msgs::msg::Point point1,
  const geometry_msgs::msg::Point point2)
{
  geometry_msgs::msg::Vector3 vector;

  // Calcular las componentes del vector
  vector.x = point2.x - point1.x;
  vector.y = point2.y - point1.y;

  // Calcular la longitud del vector
  double vector_length = std::sqrt(vector.x * vector.x + vector.y * vector.y);

  // Limitar la velocidad lineal máxima
  if (vector_length > this->MAX_LIN_VEL) {
      double factor = this->MAX_LIN_VEL / vector_length;
      vector.x *= factor;
      vector.y *= factor;
  }

  return vector;
}

ArrayVector3d
ReynoldRulesNode::nav_2_point_rule()
{
  // Verificar si hay un nuevo punto objetivo
  if (this->target_point.x != this->prev_point.x || this->target_point.y != this->prev_point.y) {
    // Calcular el punto de inicio como la posición promedio de los robots
    geometry_msgs::msg::Point start;
    for (const auto& robot : this->robots_) {
      start.x += robot->pose.pose.position.x;
      start.y += robot->pose.pose.position.y;
    }
    start.x /= this->NUMBER_DRONES;
    start.y /= this->NUMBER_DRONES;

    // Encontrar el waypoint más cercano al inicio y al objetivo
    double dist = -1; // Inicializar con el valor máximo de double
    geometry_msgs::msg::Point closer_2_start;
    for (const auto& wp : this->waypoints_) { // Iterar sobre la lista de waypoints
      double current_dist = get_distance(start, wp); // Calcular la distancia actual
      if (current_dist < dist || dist < 0) {
          dist = current_dist;
          closer_2_start = wp; // Actualizar el punto más cercano
      }
    }
    
    dist = -1; // Restart the calue of dist
    geometry_msgs::msg::Point closer_2_target;
    for (const auto& wp : this->waypoints_) { // Iterar sobre la lista de waypoints
      double current_dist = get_distance(start, wp); // Calcular la distancia actual
      if (current_dist < dist || dist < 0) {
          dist = current_dist;
          closer_2_target = wp; // Actualizar el punto más cercano
      }
    }

    // Encontrar el camino a través de los waypoints
    this->path_ = findPathThroughWaypoints(closer_2_start, closer_2_target);
    this->path_.push_back(this->target_point); // Añadir el punto objetivo al camino

    if (!this->path_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Ruta encontrada:");
      for (const auto& p : this->path_) {
          RCLCPP_INFO(this->get_logger(), "(%f, %f)", p.x, p.y);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "No se encontró una ruta disponible.");
    }
  }

  // Verificar si el enjambre ha llegado al waypoint actual
  geometry_msgs::msg::Point average_position;
  for (const auto& robot : this->robots_) {
    average_position.x += robot->pose.pose.position.x;
    average_position.y += robot->pose.pose.position.y;
  }
  average_position.x /= this->NUMBER_DRONES;
  average_position.y /= this->NUMBER_DRONES;

  if (get_distance(average_position, this->path_.front()) < this->DIST_THRESHOLD) {
    if (this->path_.front() != this->target_point) {
      this->path_.erase(this->path_.begin()); // Eliminar waypoint del camino
    }
  }

  // Calcular los vectores de navegación para los robots hacia el waypoint actual del camino
  std::vector<geometry_msgs::msg::Vector3> path_;
  ArrayVector3d nav_2_point_vectors;
  for (const auto& robot : this->robots_) {
    geometry_msgs::msg::Vector3 vec = vector_2_points(robot->pose.pose.position, this->path_.front());
    nav_2_point_vectors.vectors.push_back(vec);
  }

  // Actualizar el punto objetivo anterior
  this->prev_point = this->target_point;

  // Publicar los vectores de navegación
  return nav_2_point_vectors;
}

ArrayVector3d
ReynoldRulesNode::avoidance_rule()
{
  ArrayVector3d avoidance_vectors;

  for (int i = 0; i < NUMBER_DRONES; i++) {
    avoidance_vectors.vectors.push_back(sum_weighted_repellent_vectors(i));
  }

  return avoidance_vectors;
}

geometry_msgs::msg::Vector3
ReynoldRulesNode::sum_weighted_repellent_vectors(int robot_index)
{
  // Find obstacles in the view range and compute the weighted sum of the distance vectors.
  // The weight for each distance vector is (-1 / distance) to simulate a repellent force.

  geometry_msgs::msg::Vector3 v3;

  if (!robots_[robot_index]) {
      return v3;
  }

  auto robot_pose = robots_[robot_index]->pose.pose;
  std::vector<geometry_msgs::msg::Point> obstacles = find_obstacles(robot_pose);
  if (size(obstacles) == 0) {
      return v3;
  }

  std::vector<double> robot_position = {robot_pose.position.x, robot_pose.position.y};


  std::vector<double> v = {0.0, 0.0};
  for (geometry_msgs::msg::Point obstacle : obstacles) {
    std::vector<double> obstacle_arr = {obstacle.x, obstacle.y};
    std::vector<double> opposite_vector;
    opposite_vector[0] = robot_position[0] - obstacle_arr[0];
    opposite_vector[1] = robot_position[1] - obstacle_arr[1];
    double distance = compute_norm(opposite_vector);
    if (distance != 0) {
        v[0] += opposite_vector[0] / distance;
        v[1] += opposite_vector[1] / distance;
    }
  }

  v[0] = v[0] / size(obstacles);
  v[1] = v[1] / size(obstacles);

  v3.x = v[0];
  v3.y = v[1];
  return v3;
}

std::vector<geometry_msgs::msg::Point>
ReynoldRulesNode::find_obstacles(const geometry_msgs::msg::Pose robot_pose)
{
  std::vector<geometry_msgs::msg::Point> obstacles;
  if (!map_) {
    return obstacles;
  }

  double robot_x = robot_pose.position.x;
  double robot_y = robot_pose.position.y;
  auto robot_q = robot_pose.orientation;

  double robot_yaw = 2 * atan2(robot_q.z, robot_q.w);

  double min_angle = view_split_ * (M_PI / 180.0);
  double max_angle = (view_split_ + view_angle_) * (M_PI / 180.0);

  double radial_step_size = map_->info.resolution;
  int n_radial = static_cast<int>(view_range_ / radial_step_size);
  auto s = ANGULAR_STEPSIZE;
  std::vector<int> angular_range;
  for (int i = int(-max_angle / s); i < int(max_angle / s); ++i) {
    if (i < int(-min_angle / s) || i >= int(min_angle / s)) {
      angular_range.push_back(i);
    }
  }

  for (auto a : angular_range) {
      double alpha = robot_yaw + a * ANGULAR_STEPSIZE;
      double cos_alpha = cos(alpha);
      double sin_alpha = sin(alpha);
      for (int r = 1; r < n_radial; r++) {
          double distance = r * radial_step_size;
          geometry_msgs::msg::Point p;
          p.x = robot_x + distance * cos_alpha;
          p.y = robot_y + distance * sin_alpha;
          // ** TODO p.z (?)
          if (map_lookup(p)){
              obstacles.push_back(p);
          }
      }
  }
  return obstacles;
}

bool
ReynoldRulesNode::map_lookup(geometry_msgs::msg::Point& pos)
{
  // Return True if the position is within the map and there is an obstacle, else False

  if (!map_) {
  return false;
  }
    int i = static_cast<int>((pos.y - map_->info.origin.position.y) / map_->info.resolution);
    int j = static_cast<int>((pos.x - map_->info.origin.position.x) / map_->info.resolution);
    int index = map_->info.width * i + j;
    if (index < 0 || index >= static_cast<int>(map_->data.size())) {
      return false;
    }
    return map_->data[index] == 100;
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
