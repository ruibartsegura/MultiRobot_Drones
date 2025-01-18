#include "reynold_rules/reynold_rules_node.hpp"

#include <algorithm>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "reynold_rules/reynold_rules_node.hpp"
#include "std_msgs/msg/int32.hpp"
#include <geometry_msgs/msg/quaternion.hpp>

#include <iostream>
#include <queue>
#include <set>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Vector3;

namespace {
[[nodiscard]] std::vector<std::vector<double>> parseTopology(const std::string& s)
{
	if (s.empty()) {
		throw std::runtime_error{"Cannot parse empty string."};
	}

	std::vector<std::vector<double>> topology;
	std::stringstream outerStream(s);
	while (outerStream.good()) {
		std::string line;
		std::getline(outerStream, line, ';');

		std::vector<double> v;
		std::stringstream ss(line);
		for (double i; ss >> i;) {
			v.push_back(i);
			if (ss.peek() == ',') {
				ss.ignore();
			}
		}

		topology.push_back(v);
		if (outerStream.peek() == ';') {
			outerStream.ignore();
		}
	}

	return topology;
}

[[nodiscard]] Point add(Point p, const Vector3& v)
{
	p.x += v.x;
	p.y += v.y;
	p.z += v.z;
	return p;
}

[[nodiscard]] Vector3 mul(double scalar, Vector3 v)
{
	v.x *= scalar;
	v.y *= scalar;
	v.z *= scalar;
	return v;
}

[[nodiscard]] double get_distance(geometry_msgs::msg::Point pos1, geometry_msgs::msg::Point pos2)
{
	auto x = pos1.x - pos2.x;
	auto y = pos1.y - pos2.y;
	auto z = pos1.z - pos2.z;
	// std::cout << "DIST" << sqrt(x * x + y * y + z * z) << std::endl;
	return sqrt(x * x + y * y + z * z);
}

} // namespace

namespace reynold_rules
{

double
compute_norm(const std::vector<double>& vec)
{
  double sum_of_squares = std::accumulate(vec.begin(), vec.end(), 0.0,
    [](double sum, double val) { return sum + val * val; }); 
  return sqrt(sum_of_squares);
}

ReynoldRulesNode::ReynoldRulesNode()
: Node("reynold_rules_node"), map_(nullptr)
{
	//formation_control_setup();

	// Parameters
	declare_parameter("NUMBER_DRONES", NUMBER_DRONES);
	get_parameter("NUMBER_DRONES", NUMBER_DRONES);

	declare_parameter("MIN_LIN_VEL", MIN_LIN_VEL);
	get_parameter("MIN_LIN_VEL", MIN_LIN_VEL);

	declare_parameter("MAX_LIN_VEL", MAX_LIN_VEL);
	get_parameter("MAX_LIN_VEL", MAX_LIN_VEL);
		
	declare_parameter("DIST_THRESHOLD", DIST_THRESHOLD);
	get_parameter("DIST_THRESHOLD", DIST_THRESHOLD);

	declare_parameter("MAX_VEL_DIFF_FACTOR", MAX_VEL_DIFF_FACTOR);
	get_parameter("MAX_VEL_DIFF_FACTOR", MAX_VEL_DIFF_FACTOR);
	
	declare_parameter("view_range", view_range_);
	get_parameter("view_range", view_range_);
	
	declare_parameter("view_angle", view_angle_);
	get_parameter("view_angle", view_angle_);

	declare_parameter("view_split", view_split_);
	get_parameter("view_split", view_split_);

	declare_parameter("separation_weight_", separation_weight_);
	get_parameter("separation_weight_", separation_weight_);

	declare_parameter("cohesion_weight_", cohesion_weight_);
	get_parameter("cohesion_weight_", cohesion_weight_);

	declare_parameter("alignment_weight_", alignment_weight_);
	get_parameter("alignment_weight_", alignment_weight_);

	declare_parameter("obstacle_avoidance_weight_", obstacle_avoidance_weight_);
	get_parameter("obstacle_avoidance_weight_", obstacle_avoidance_weight_);

	declare_parameter("nav2point_weight_", nav2point_weight_);
	get_parameter("nav2point_weight_", nav2point_weight_);
	
	this->declare_parameter<std::vector<double>>("target_point", {0.0, 0.0, 0.0});

	// Obtener el parámetro y convertirlo a geometry_msgs::msg::Point
	std::vector<double> target_point_vec;
	this->get_parameter("target_point", target_point_vec);

	if (target_point_vec.size() == 3) {
		target_point_.x = target_point_vec[0];
		target_point_.y = target_point_vec[1];
		target_point_.z = target_point_vec[2];
		RCLCPP_INFO(this->get_logger(), "Target point initialized: x=%.2f, y=%.2f, z=%.2f",
					target_point_.x, target_point_.y, target_point_.z);
	}

	// List of waypoint to de nav_2_point
	for (int x = -4; x <= 4; x += 2) {
		for (int y = -4; y <= 4; y += 2) {
			geometry_msgs::msg::Point wp;
			wp.x = x;
			wp.y = y;
			waypoints_.push_back(wp);
		}
	}

	std::string navigationMethod;
	declare_parameter("navigation_method", navigationMethod);
	get_parameter("navigation_method", navigationMethod);

	std::string topology;
	declare_parameter("topology", topology);
	get_parameter("topology", topology);
	try {
		topology_ = parseTopology(topology);
		RCLCPP_INFO(get_logger(), "Parsed topology: %s", topology.c_str());
	} catch (const std::exception& e) {
		// fill topology with zeros
		topology_ = {static_cast<size_t>(NUMBER_DRONES), std::vector<double>(NUMBER_DRONES)};
		RCLCPP_ERROR(get_logger(), "Communication disabled, parsing failed: %s", e.what());
	}

	// Make sure the array of the odom for the drones is of the correct size
	if (robots_.size() <= static_cast<size_t>(NUMBER_DRONES)) {
		robots_.resize(NUMBER_DRONES); // Redimensiona si es necesario
	}

	for (int i = 1; i <= NUMBER_DRONES; i++) {
		std::string odom_topic = "/cf_" + std::to_string(i) + "/odom";
		std::string vel_topic  = "/cf_" + std::to_string(i) + "/cmd_vel";

		publishers_.push_back(create_publisher<geometry_msgs::msg::Twist>(vel_topic, 10));
		subscribers_.push_back(create_subscription<nav_msgs::msg::Odometry>(
		        odom_topic,
		        10,
		        std::bind(&ReynoldRulesNode::odom_callback, this, std::placeholders::_1)));
	}

	map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
	   "/map",
	   10,
	   std::bind(&ReynoldRulesNode::map_callback, this, std::placeholders::_1)
	);

	timer_ = create_wall_timer(500ms, std::bind(&ReynoldRulesNode::control_cycle, this));
}

void
ReynoldRulesNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr data)
{
	std::string number = data->child_frame_id.substr(std::strlen("cf_"));

	// Asignación de SharedPtr en el vector
	int drone_number = std::stoi(number);

	robots_[drone_number - 1] = data;
	// std::cout << "H " << data->pose.pose.position.z << std::endl;
  	if (data->pose.pose.position.z > 0.3) {
		READY = true;
	}
}

void
ReynoldRulesNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr data)
{
  // if (this->map_ == nullptr) {  // Usamos 'this->' para acceder a la variable miembro
  this->map_ = data;  // Asigna el primer mapa recibido a map_
  checkPathsBetweenWaypoints();
  // }
}

geometry_msgs::msg::Vector3
ReynoldRulesNode::calc_sep_vector(geometry_msgs::msg::Point position, int num)
{
  geometry_msgs::msg::Vector3 repulsive_vector; //k is the cte of force
  double k = 0.01;

  for (int i = 0; i < NUMBER_DRONES; i++){
    // Avoid calculating the robot's own vector
    if (i != num){
      // Get the distance betwen the robots
      double dist = get_distance(position, robots_[i]->pose.pose.position);

      // Check if the distance is in the radious
      if ((dist > 0.0) && (dist < view_range_)){
        // Get the x, y coords of the vector
        auto x = position.x - robots_[i]->pose.pose.position.x;
        auto y = position.y - robots_[i]->pose.pose.position.y;
        auto z = position.z - robots_[i]->pose.pose.position.z;

        // Normalize the vector
        auto norm = sqrt(x * x + y * y + z * z);

        double direction[3];
        if (norm > 0){
            direction[0] = x / norm;
            direction[1] = y / norm;
            direction[1] = z / norm;
        } else {
            direction[0] = 0.00;
            direction[1] = 0.00;
            direction[2] = 0.00;
        }

        // Magnitude od the repulsice vector
        auto magnitude = k / (dist * dist);

        // Sum to the total the repulsive vector of robot_i
        repulsive_vector.x += magnitude * direction[0];
        repulsive_vector.y += magnitude * direction[1];
        repulsive_vector.z += magnitude * direction[2];
      }
    }
  }
  return repulsive_vector;
}

std::vector<geometry_msgs::msg::Vector3>
ReynoldRulesNode::separation_rule()
{
  std::vector<geometry_msgs::msg::Vector3> separation_vectors;
  for (int i = 0; i < NUMBER_DRONES; i++) {
    geometry_msgs::msg::Vector3 vec = calc_sep_vector(this->robots_[i]->pose.pose.position, i);
    separation_vectors.push_back(vec); // Agregar al vector del mensaje
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

std::vector<geometry_msgs::msg::Vector3>
ReynoldRulesNode::aligment_rule()
{
  geometry_msgs::msg::Vector3 avg_velocity = calc_average_velocity();
  std::vector<geometry_msgs::msg::Vector3> alignment_vectors;

  for (nav_msgs::msg::Odometry::SharedPtr robot : robots_) {
    geometry_msgs::msg::Vector3 alignment_vector;
    alignment_vector.x = avg_velocity.x - robot->twist.twist.linear.x;
    alignment_vector.y = avg_velocity.y - robot->twist.twist.linear.y;
    alignment_vectors.push_back(alignment_vector);
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

std::vector<geometry_msgs::msg::Vector3>
ReynoldRulesNode::cohesion_rule()
{
  std::vector<geometry_msgs::msg::Vector3> cohesion_vectors;

  for (nav_msgs::msg::Odometry::SharedPtr robot : robots_) {
      geometry_msgs::msg::Vector3 cohesion_vector = calc_cohesion_vector(robot->pose.pose.position);
      cohesion_vectors.push_back(cohesion_vector);
  }

  return cohesion_vectors;
}

// Find the neighbors of a waypoint
std::vector<geometry_msgs::msg::Point>
ReynoldRulesNode::findNeighbors(
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
bool
ReynoldRulesNode::isPathClear(
	const std::pair<int, int>& start,
	const std::pair<int, int>& end)
{
  if (!this->map_) return false;

  int startI = static_cast<int>((start.second - this->map_->info.origin.position.y) / this->map_->info.resolution);
  int startJ = static_cast<int>((start.first - this->map_->info.origin.position.x) / this->map_->info.resolution);
  int endI = static_cast<int>((end.second - this->map_->info.origin.position.y) / this->map_->info.resolution);
  int endJ = static_cast<int>((end.first - this->map_->info.origin.position.x) / this->map_->info.resolution);

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

void
ReynoldRulesNode::recalculatePath()
{
	// Calcular el punto de inicio como la posición promedio de los robots
	geometry_msgs::msg::Point start;
	for (const auto& robot : this->robots_) {
		start.x += robot->pose.pose.position.x;
		start.y += robot->pose.pose.position.y;
	}
	start.x /= this->NUMBER_DRONES;
	start.y /= this->NUMBER_DRONES;

	std::cout << "Start x" << start.x << std::endl;
	std::cout << "Start y" << start.y << std::endl;

	// Encontrar el waypoint más cercano al inicio y al objetivo
	double dist = -1; // Inicializar con el valor máximo de double
	geometry_msgs::msg::Point closer_2_start;
	for (const auto& wp : this->waypoints_) {              // Iterar sobre la lista de waypoints
		double current_dist = get_distance(start, wp); // Calcular la distancia actual
		if (current_dist < dist || dist < 0) {
			dist           = current_dist;
			closer_2_start = wp; // Actualizar el punto más cercano
		}
	}

	dist = -1; // Restart the calue of dist
	geometry_msgs::msg::Point closer_2_target;
	for (const auto& wp : this->waypoints_) {              // Iterar sobre la lista de waypoints
		double current_dist = get_distance(target_point_, wp); // Calcular la distancia actual
		if (current_dist < dist || dist < 0) {
			dist            = current_dist;
			closer_2_target = wp; // Actualizar el punto más cercano
		}
	}

	std::cout << "Close start x" << closer_2_start.x << std::endl;
	std::cout << "Close start y" << closer_2_start.y << std::endl;
	std::cout << "Close target x" << closer_2_target.x << std::endl;
	std::cout << "Close target y" << closer_2_target.y << std::endl;

	// Encontrar el camino a través de los waypoints
	this->path_ = findPathThroughWaypoints(closer_2_start, closer_2_target);
	this->path_.push_back(this->target_point_); // Añadir el punto objetivo al camino

	if (!this->path_.empty()) {
		RCLCPP_INFO(this->get_logger(), "Ruta encontrada:");
		for (const auto& p : this->path_) {
			RCLCPP_INFO(this->get_logger(), "(%f, %f)", p.x, p.y);
		}
	} else {
		RCLCPP_WARN(this->get_logger(), "No se encontró una ruta disponible.");
	}
}

std::vector<geometry_msgs::msg::Point>
ReynoldRulesNode::findPathThroughWaypoints(
        const geometry_msgs::msg::Point& start,
		const geometry_msgs::msg::Point& target)
{
	using Position2D = std::pair<int, int>;

	Position2D startPos  = {static_cast<int>(start.x), static_cast<int>(start.y)};
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
		auto neighbors        = findNeighbors(waypoints_, currentWp);

		for (const auto& neighbor : neighbors) {
			Position2D neighborPos = {static_cast<int>(neighbor.x),
			                          static_cast<int>(neighbor.y)};
			if (visited.find(neighborPos) == visited.end() &&
			    isPathClear(currentPos, neighborPos)) {
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
	const geometry_msgs::msg::Point point2,
    std::optional<double> max_length)
{
	geometry_msgs::msg::Vector3 vector;

	// Calcular las componentes del vector
	vector.x = point2.x - point1.x;
	vector.y = point2.y - point1.y;

	if (max_length) {
		// Calcular la longitud del vector
		const double vector_length = std::sqrt(vector.x * vector.x + vector.y * vector.y);

		// Limitar la velocidad lineal máxima
		if (vector_length > max_length.value()) {
			double factor = max_length.value() / vector_length;
			vector.x *= factor;
			vector.y *= factor;
		}
	}
	return vector;
}

std::vector<geometry_msgs::msg::Vector3>
ReynoldRulesNode::nav_2_point_rule()
{
	std::vector<geometry_msgs::msg::Vector3> nav_2_point_vectors;

	if (this->navigationMethod_ == NavigationMethod::RosParam) {
		// Verificar si hay un nuevo punto objetivo
		if (this->target_point_.x != this->prev_point.x ||
		    this->target_point_.y != this->prev_point.y) {
			std::cout << "Primer Calculo ruta" << std::endl;
			recalculatePath();
			this->prev_point = this->target_point_;
		}

		RCLCPP_INFO(get_logger(), "nav_2_point_rule: 1");
		// Verificar si el enjambre ha llegado al waypoint actual
		geometry_msgs::msg::Point average_position;
		for (const auto& robot : this->robots_) {
			average_position.x += robot->pose.pose.position.x;
			average_position.y += robot->pose.pose.position.y;
			average_position.z += robot->pose.pose.position.z;
		}
		average_position.x /= this->NUMBER_DRONES;
		average_position.y /= this->NUMBER_DRONES;
		average_position.z /= this->NUMBER_DRONES;

		RCLCPP_WARN(get_logger(), "nav_2_point_rule: 2");
		if (get_distance(average_position, this->path_.front()) < this->DIST_THRESHOLD) {
			if (this->path_.front() != this->target_point_) {
				this->path_.erase(this->path_.begin()); // Eliminar waypoint del camino
			}
		}

		RCLCPP_INFO(get_logger(), "nav_2_point_rule: 3");
		// Calcular los vectores de navegación para los robots hacia el waypoint actual del
		// camino
		for (const auto& robot : this->robots_) {
			const auto vec = vector_2_points(robot->pose.pose.position,
			                                 this->path_.front(),
			                                 this->MAX_LIN_VEL);
			nav_2_point_vectors.push_back(vec);
		}
		RCLCPP_INFO(get_logger(), "end");
	} else if (this->navigationMethod_ == NavigationMethod::Rendezvous) {
		RCLCPP_WARN(this->get_logger(), "Rendezvous");
		rendezvous_protocol();

		if (this->paths_.size() != this->robots_.size()) {
			throw std::runtime_error{"robots_ and paths_ sizes don't match"};
		}

		for (size_t i = 0; i < this->robots_.size(); i++) {
			auto& path      = paths_[i];
			const auto& pos = robots_[i]->pose.pose.position;
			if ((path.size() > 1) &&
			    (get_distance(pos, path.front()) < this->DIST_THRESHOLD)) {
				path.erase(path.begin());
			}

			auto vec = vector_2_points(pos, path.front(), this->MAX_LIN_VEL);
			nav_2_point_vectors.push_back(vec);
		}
	}

	// Publicar los vectores de navegación
	return nav_2_point_vectors;
}

std::vector<geometry_msgs::msg::Vector3>
ReynoldRulesNode::avoidance_rule()
{
  void();
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

double
ReynoldRulesNode::calc_length(const geometry_msgs::msg::Vector3& vector)
{
	return std::sqrt(vector.x * vector.x + vector.y * vector.y);
}

void
ReynoldRulesNode::control_cycle()
{
	if (READY) {
		std::vector<std::vector<geometry_msgs::msg::Vector3>> rules = {
		        // separation_rule(),
		        // aligment_rule(),
		        // cohesion_rule(),
		        nav_2_point_rule(),
		        // avoidance_rule()
		};

		std::vector<double> weights = {
		        // separation_weight_,
				nav2point_weight_,
		        // obstacle_avoidance_weight_,
		        // cohesion_weight_,
		        // alignment_weight_
		};

		// if (formation_type_ != NONE) {
		// 	rules.push_back(formation_control());
		// 	weights.push_back(formation_weight_);
		// }

		for (size_t i = 0; i < static_cast<size_t>(NUMBER_DRONES); ++i) {
			geometry_msgs::msg::Vector3 total_vector;

			for (size_t j = 0; j < rules.size(); ++j) {
				geometry_msgs::msg::Vector3 new_vector;
				new_vector.x = weights[j] * rules[j][i].x;
				new_vector.y = weights[j] * rules[j][i].y;

				total_vector.x += new_vector.x;
				total_vector.y += new_vector.y;

				if (calc_length(new_vector) > MAX_LIN_VEL) {
					break;
				}
			}

			geometry_msgs::msg::Twist vel;
			vel.linear.x = std::min(linear_mult_ * total_vector.x, MAX_LIN_VEL);
			vel.linear.y = std::min(linear_mult_ * total_vector.y, MAX_LIN_VEL);
	
			// Imprimir los valores de velocidad de los drones
			// std::cout << "VELOCITY " << i+1 << ": x=" << vel.linear.x
			//           << ", y=" << vel.linear.y << std::endl;

			publishers_[i]->publish(vel);
		}
	} else {
		std::cout << "No empieza" << std::endl;

	}
}

void
ReynoldRulesNode::checkPathsBetweenWaypoints() {
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

void
ReynoldRulesNode::rendezvous_protocol()
{
	// Implement a consensus-based rendezvous protocol. vector of a rendezvous protocol is to
	// gather the robots in a common position in space. Test the behavior for at least two fixed
	// directed communication topologies for random initial positions of the robots.
	this->paths_.clear();

	for (size_t i = 0; i < robots_.size(); i++) {
		geometry_msgs::msg::Point x{};
		for (size_t j = 0; j < robots_.size(); j++) {
			// implement the consensus equation
			const auto a_ij = topology_.at(i).at(j);
			const auto d    = vector_2_points(robots_[i]->pose.pose.position,
                                                       robots_[j]->pose.pose.position);

			// x += a_ij * d
			x = add(x, mul(a_ij, d));
		}

		// update the robot's navigation to x
		auto path = findPathThroughWaypoints(robots_[i]->pose.pose.position, x);
		path.push_back(x);
		this->paths_.emplace_back(path);
	}
}

std::vector<geometry_msgs::msg::Point>	
ReynoldRulesNode::get_formation_points(int type, float side_length)
{
// Create vector with point for each robot in the desired figure
	float dist;

	std::vector<geometry_msgs::msg::Point> points;
	geometry_msgs::msg::Point point1, point2, point3, point4;

	switch (type) {
	case LINE: // (0,0); (d,0); (2d,0); (3d,0)...
		dist = side_length / NUMBER_DRONES;
		for (int i = 0; i < NUMBER_DRONES; i++) {

			geometry_msgs::msg::Point point;
			point.x = dist * i;
			point.y = 0;

			points.push_back(point);
		}
		break;

	case TRIANGLE: //(0,0); (0,l); (l/2, lcos(pi/6)); (l/2, l/2cos(pi/6))
		point1.x = 0;
		point1.y = 0;

		point2.x = side_length;
		point2.y = 0;

		point3.x = side_length / 2;
		point3.y = side_length * std::cos(M_PI / 6);

		point4.x = point3.x;
		point4.y = point3.y / 2;

		points = {point1, point2, point3, point4};
		break;

	case SQUARE: //(0,0); (l,0); (l, l); (0, l)
		point1.x = point1.y = point2.y = point4.x = 0;
		point2.x = point3.x = point3.y = point4.y = side_length;

		points = {point1, point2, point3, point4};
	}

	return points;
}

/** Sets matrix containing relative position of each robot
	using points of the figure to represent */
void
ReynoldRulesNode::set_formation_matrix(
	std::vector<geometry_msgs::msg::Point> formation_points)
{
	// Set matrix initially to all 0
	formation_matrix_ = {static_cast<size_t>(NUMBER_DRONES), std::vector<geometry_msgs::msg::Point>(NUMBER_DRONES)};

	for (int i = 0; i < NUMBER_DRONES; i++) {
		for (int j = 0; j < NUMBER_DRONES; j++) {

			formation_matrix_[i][j].x = formation_points[j].x - formation_points[i].x;
			formation_matrix_[i][j].y = formation_points[j].y - formation_points[i].y;
		}
	}
}

void
ReynoldRulesNode::formation_control_setup()
{
//  Sets up everything neccesary for formation protocol if it is activated
	float side_length;

	this->declare_parameter("formation_weight", 0.0);
	this->declare_parameter("formation_type", 0);
	this->declare_parameter("side_length", 0.0);

	this->get_parameter("formation_weight", formation_weight_);
	this->get_parameter("formation_type", formation_type_);
	this->get_parameter("side_length", side_length);

	// Dont create unneccessay matrix if not needed
	if (formation_type_ == NONE) {return;}

	std::cout << "formation type: " << std::to_string(formation_type_).c_str()
			<< "\nside length: " << std::to_string(side_length).c_str() << std::endl;

	set_formation_matrix(get_formation_points(formation_type_, side_length));
}

std::vector<geometry_msgs::msg::Vector3>
ReynoldRulesNode::formation_control()
{
	std::vector<geometry_msgs::msg::Vector3> formation_vectors;

	for (int i = 0; i < NUMBER_DRONES; i++) {
		geometry_msgs::msg::Vector3 final_vector;

		for (int j = 0; j < NUMBER_DRONES; j++) {

			// Access matrix trasposed (Personal Implementation)
			const auto a_ij = topology_.at(j).at(i);
			geometry_msgs::msg::Vector3 vector;

			vector.x = a_ij * (robots_[j]->pose.pose.position.x
							- robots_[i]->pose.pose.position.x
							+ formation_matrix_[j][i].x);
		
			vector.y = a_ij * (robots_[j]->pose.pose.position.y
							- robots_[i]->pose.pose.position.y
							+ formation_matrix_[j][i].y);
			
			final_vector.x = final_vector.x + vector.x;
			final_vector.y = final_vector.y + vector.y;
		}

		formation_vectors.push_back(final_vector);
	}

	return formation_vectors;
}

} //  namespace reynold_rules
