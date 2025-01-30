#include "reynold_rules/reynold_rules_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/srv/get_map.hpp"
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

template <typename T>
[[nodiscard]] T add(T p, const Vector3& v)
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

[[nodiscard]] Vector3 div(Vector3 v, double scalar)
{
	v.x /= scalar;
	v.y /= scalar;
	v.z /= scalar;
	return v;
}

[[nodiscard]] double get_distance(geometry_msgs::msg::Point pos1, geometry_msgs::msg::Point pos2)
{
	auto x = pos1.x - pos2.x;
	auto y = pos1.y - pos2.y;
	auto z = pos1.z - pos2.z;
	return sqrt(x * x + y * y + z * z);
}

[[nodiscard]] double get_distance_2d(geometry_msgs::msg::Point pos1, geometry_msgs::msg::Point pos2)
{
	auto x = pos1.x - pos2.x;
	auto y = pos1.y - pos2.y;
	return sqrt(x * x + y * y);
}

} // namespace

namespace reynold_rules {

double compute_norm(const std::vector<double>& vec)
{
	double sum_of_squares = std::accumulate(vec.begin(), vec.end(), 0.0, [](double sum, double val) {
		return sum + val * val;
	});
	return sqrt(sum_of_squares);
}

ReynoldRulesNode::ReynoldRulesNode() : Node("reynold_rules_node"), map_(nullptr)
{
	formation_control_setup();

	// Parameters
	declare_parameter("NUMBER_DRONES", NUMBER_DRONES);
	get_parameter("NUMBER_DRONES", NUMBER_DRONES);
  RCLCPP_INFO(get_logger(), "NUMBER_DRONES: %ld", NUMBER_DRONES);

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

	declare_parameter("separation_weight", separation_weight_);
	get_parameter("separation_weight", separation_weight_);

	declare_parameter("cohesion_weight", cohesion_weight_);
	get_parameter("cohesion_weight", cohesion_weight_);

	declare_parameter("alignment_weight", alignment_weight_);
	get_parameter("alignment_weight", alignment_weight_);

	declare_parameter("obstacle_avoidance_weight", obstacle_avoidance_weight_);
	get_parameter("obstacle_avoidance_weight", obstacle_avoidance_weight_);

	declare_parameter("nav2point_weight", nav2point_weight_);
	get_parameter("nav2point_weight", nav2point_weight_);

	auto control_cycle_period_ms = control_cycle_period_.count();
	declare_parameter("control_cycle_period_ms", control_cycle_period_ms);
	get_parameter("control_cycle_period_ms", control_cycle_period_ms);
	control_cycle_period_ = std::chrono::milliseconds{control_cycle_period_ms};

	declare_parameter("rendezvous_recalc_period", rendezvous_recalc_period_);
	get_parameter("rendezvous_recalc_period", rendezvous_recalc_period_);

	this->declare_parameter<std::vector<double>>("target_point", {0.0, 0.0, 0.0});

	// Obtener el parámetro y convertirlo a geometry_msgs::msg::Point
	std::vector<double> target_point_vec;
	this->get_parameter("target_point", target_point_vec);

	if (target_point_vec.size() == 3) {
		target_point_.x = target_point_vec[0];
		target_point_.y = target_point_vec[1];
		target_point_.z = target_point_vec[2];
		RCLCPP_INFO(this->get_logger(),
		            "Target point initialized: x=%.2f, y=%.2f, z=%.2f",
		            target_point_.x,
		            target_point_.y,
		            target_point_.z);
	}
	// The drone can't be underground so in the first comparation the result is going to be
	// always false
	this->prev_point.x = 0;
	this->prev_point.y = 0;
	this->prev_point.z = -1;

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
	if (navigationMethod == "Rendezvous") {
		navigationMethod_ = NavigationMethod::Rendezvous;
	}

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
		ready_.resize(NUMBER_DRONES);
	}

	for (int i = 0; i < NUMBER_DRONES; i++) {
		std::string odom_topic = "/cf_" + std::to_string(i + 1) + "/odom";
		std::string vel_topic  = "/cf_" + std::to_string(i + 1) + "/cmd_vel";

		publishers_.push_back(create_publisher<geometry_msgs::msg::Twist>(vel_topic, 10));
		subscribers_.push_back(create_subscription<nav_msgs::msg::Odometry>(
		        odom_topic,
		        10,
		        std::bind(&ReynoldRulesNode::odom_callback, this, std::placeholders::_1)));

		ready_[i] = false;
	}

	// map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
	//         "/map", 10, std::bind(&ReynoldRulesNode::map_callback, this,
	//         std::placeholders::_1));

	// request_map();
	// if(map_) {
	//   checkPathsBetweenWaypoints();
	//   RCLCPP_INFO(get_logger(), "Map received, starting control_cycle");
	//   timer_ = create_wall_timer(control_cycle_period_,
	//   std::bind(&ReynoldRulesNode::control_cycle, this));
	// } else {
	//   RCLCPP_INFO(get_logger(), "Node created, waiting for map...");
	// }

	timer_ = create_wall_timer(control_cycle_period_,
	                           std::bind(&ReynoldRulesNode::control_cycle, this));
}

void ReynoldRulesNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr data)
{
	std::string number = data->child_frame_id.substr(std::strlen("cf_"));

	// Asignación de SharedPtr en el vector
	int drone_number = std::stoi(number);

	if (drone_number > robots_.size()) {
		RCLCPP_WARN(get_logger(), "Discard odom for drone %s", data->child_frame_id.c_str());
    return;
	}

	robots_[drone_number - 1] = data;

	if (data->pose.pose.position.z > HEIGHT) {
		ready_[drone_number - 1] = true;
	}
}

void ReynoldRulesNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr data)
{
	const bool first = !map_;
	this->map_       = data;
	//checkPathsBetweenWaypoints();

	if (first) {
		RCLCPP_INFO(get_logger(), "Map received, starting control_cycle");
		timer_ = create_wall_timer(control_cycle_period_,
		                           std::bind(&ReynoldRulesNode::control_cycle, this));
	}
}

void ReynoldRulesNode::request_map() {}

geometry_msgs::msg::Vector3 ReynoldRulesNode::calc_sep_vector(geometry_msgs::msg::Point position, int num)
{
	geometry_msgs::msg::Vector3 repulsive_vector; // k is the cte of force
	double k = 0.01;

	for (int i = 0; i < NUMBER_DRONES; i++) {
		// Avoid calculating the robot's own vector
		if (i != num) {
			// Get the distance betwen the robots
			double dist = get_distance(position, odom(i)->pose.pose.position);

			// Check if the distance is in the radious
			if ((dist > 0.0) && (dist < view_range_)) {
				// Get the x, y coords of the vector
				auto x = position.x - odom(i)->pose.pose.position.x;
				auto y = position.y - odom(i)->pose.pose.position.y;
				auto z = position.z - odom(i)->pose.pose.position.z;

				// Normalize the vector
				auto norm = sqrt(x * x + y * y + z * z);

				double direction[3];
				if (norm > 0) {
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

std::vector<geometry_msgs::msg::Vector3> ReynoldRulesNode::separation_rule()
{
	std::vector<geometry_msgs::msg::Vector3> separation_vectors;
	for (int i = 0; i < NUMBER_DRONES; i++) {
		geometry_msgs::msg::Vector3 vec = calc_sep_vector(this->odom(i)->pose.pose.position, i);
		separation_vectors.push_back(vec); // Agregar al vector del mensaje
	}

	return separation_vectors;
}

geometry_msgs::msg::Vector3 ReynoldRulesNode::calc_average_velocity()
{
	geometry_msgs::msg::Vector3 avg_velocity;

	for (nav_msgs::msg::Odometry::SharedPtr robot : robots_) {
		avg_velocity.x += robot->twist.twist.linear.x;
		avg_velocity.y += robot->twist.twist.linear.y;
		avg_velocity.z += robot->twist.twist.linear.z;
	}

	avg_velocity.x /= NUMBER_DRONES;
	avg_velocity.y /= NUMBER_DRONES;
	avg_velocity.z /= NUMBER_DRONES;

	return avg_velocity;
}

std::vector<geometry_msgs::msg::Vector3> ReynoldRulesNode::aligment_rule()
{
	geometry_msgs::msg::Vector3 avg_velocity = calc_average_velocity();
	std::vector<geometry_msgs::msg::Vector3> alignment_vectors;

	for (nav_msgs::msg::Odometry::SharedPtr robot : robots_) {
		geometry_msgs::msg::Vector3 alignment_vector;
		alignment_vector.x = avg_velocity.x - robot->twist.twist.linear.x;
		alignment_vector.y = avg_velocity.y - robot->twist.twist.linear.y;
		alignment_vector.z = avg_velocity.z - robot->twist.twist.linear.z;
		alignment_vectors.push_back(alignment_vector);
	}

	return alignment_vectors;
}

geometry_msgs::msg::Point ReynoldRulesNode::calc_average_pos(std::vector<nav_msgs::msg::Odometry> positions)
{
	geometry_msgs::msg::Point average_pos;

	for (nav_msgs::msg::Odometry position : positions) {
		average_pos.x += position.pose.pose.position.x;
		average_pos.y += position.pose.pose.position.y;
		average_pos.z += position.pose.pose.position.z;
	}

	average_pos.x = average_pos.x / positions.size();
	average_pos.y = average_pos.y / positions.size();
	average_pos.z = average_pos.z / positions.size();

	return average_pos;
}

geometry_msgs::msg::Vector3 ReynoldRulesNode::calc_cohesion_vector(geometry_msgs::msg::Point robot_pos)
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
	vector.x                              = average_pos.x - robot_pos.x;
	vector.y                              = average_pos.y - robot_pos.y;

	return vector;
}

std::vector<geometry_msgs::msg::Vector3> ReynoldRulesNode::cohesion_rule()
{
	std::vector<geometry_msgs::msg::Vector3> cohesion_vectors;

	for (nav_msgs::msg::Odometry::SharedPtr robot : robots_) {
		geometry_msgs::msg::Vector3 cohesion_vector = calc_cohesion_vector(
		        robot->pose.pose.position);
		cohesion_vectors.push_back(cohesion_vector);
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

std::vector<geometry_msgs::msg::Vector3> ReynoldRulesNode::nav_2_point_rule()
{
	std::vector<geometry_msgs::msg::Vector3> nav_2_point_vectors;
	nav_2_point_vectors.resize(NUMBER_DRONES);

	if (this->navigationMethod_ == NavigationMethod::RosParam) {
	} else if (this->navigationMethod_ == NavigationMethod::Rendezvous) {
		if (--rendezvous_counter_ <= 0) {
			rendezvous_counter_ = rendezvous_recalc_period_;
      RCLCPP_INFO(get_logger(), "Next rendezvous step");
			rendezvous_protocol();
		}

		if (this->paths_.size() != this->robots_.size()) {
			throw std::runtime_error{"robots_ and paths_ sizes don't match"};
		}

		for (size_t i = 0; i < this->robots_.size(); i++) {
			const auto& pos  = odom(i)->pose.pose.position;
			const auto& path = paths_[i];
			nav_2_point_vectors[i] = vector_2_points(pos, path.front(), this->MAX_LIN_VEL);
		}
	}

	// Publicar los vectores de navegación
	return nav_2_point_vectors;
}

std::vector<geometry_msgs::msg::Vector3> ReynoldRulesNode::avoidance_rule()
{
	std::vector<geometry_msgs::msg::Vector3> vectors;
	for (size_t i = 0; i < robots_.size(); ++i) {
		vectors.emplace_back(sum_weighted_repellent_vectors(i));
	}
	return vectors;
}

nav_msgs::msg::Odometry::SharedPtr ReynoldRulesNode::odom(size_t robot_index)
{
	if (robots_.size() <= robot_index) {
		throw std::runtime_error{"Tried to access invalid robot index " +
		                         std::to_string(robot_index)};
	}
	if (!robots_[robot_index]) {
		throw std::runtime_error{"nullptr access at robot index " + std::to_string(robot_index)};
	}
	return robots_[robot_index];
}

geometry_msgs::msg::Vector3 ReynoldRulesNode::sum_weighted_repellent_vectors(int robot_index)
{
	// Find obstacles in the view range and compute the weighted sum of the distance vectors.
	// The weight for each distance vector is (-1 / distance) to simulate a repellent force.

	if (!robots_[robot_index]) {
		return Vector3{};
	}

	const auto pose      = odom(robot_index)->pose.pose;
	const auto obstacles = find_obstacles(pose);
	if (obstacles.size() == 0) {
		return Vector3{};
	}

	Vector3 v{};
	// std::vector<double> v = {0.0, 0.0};
	for (const auto& obstacle : obstacles) {
		const auto distance = get_distance(pose.position, obstacle);
		if (distance != 0) {
			const auto opposite = vector_2_points(obstacle, pose.position);
			v                   = add(v, div(opposite, distance));
		}
	}

	v = div(v, obstacles.size());

	return v;
}

std::vector<geometry_msgs::msg::Point> ReynoldRulesNode::find_obstacles(const geometry_msgs::msg::Pose /* robot_pose */)
{
	std::vector<geometry_msgs::msg::Point> obstacles;
	return obstacles;
}

double ReynoldRulesNode::calc_length(const geometry_msgs::msg::Vector3& vector)
{
	return std::sqrt(vector.x * vector.x + vector.y * vector.y);
}

void ReynoldRulesNode::control_cycle()
{
	// Waits for all drones to be ready
	for (int i = 0; i < NUMBER_DRONES; i++) {
		if (ready_[i] == false) {
			RCLCPP_INFO(get_logger(), "Waiting for drone %d...", i + 1);
			return;
		}
	}

	std::vector<std::vector<geometry_msgs::msg::Vector3>> rules = {
	        separation_rule(),
	        // avoidance_rule(),
	        // aligment_rule(),
	        // cohesion_rule(),
	        // formation_control(),
	        nav_2_point_rule(),
	};

	std::vector<double> weights = {
	        separation_weight_,
	        // obstacle_avoidance_weight_,
	        // cohesion_weight_,
	        // alignment_weight_,
	        // formation_weight_,
	        nav2point_weight_,
	};

	for (size_t j = 0; j < rules.size(); j++) {
		if (rules[j].size() != NUMBER_DRONES) {
			throw std::runtime_error{"Vector " + std::to_string(j) + " has unexpected size" +
			                         std::to_string(rules[j].size())};
		}
	}

	for (size_t i = 0; i < static_cast<size_t>(NUMBER_DRONES); i++) {
		geometry_msgs::msg::Vector3 total_vector;

		for (size_t j = 0; j < rules.size(); j++) {
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
}


geometry_msgs::msg::Vector3 ReynoldRulesNode::vector_2_points(const geometry_msgs::msg::Point point1,
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

std::vector<geometry_msgs::msg::Point> ReynoldRulesNode::get_formation_points()
{
	// Create vector with point for each robot in the desired figure
	float dist;

	std::vector<geometry_msgs::msg::Point> points;
	geometry_msgs::msg::Point point1, point2, point3, point4;

	switch (formation_type_) {
	case LINE: // (0,0); (d,0); (2d,0); (3d,0)...
		dist = 2 * side_length_ / NUMBER_DRONES;
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

		point2.x = side_length_;
		point2.y = 0;

		point3.x = side_length_ / 2;
		point3.y = side_length_ * std::cos(M_PI / 6);

		point4.x = point3.x;
		point4.y = point3.y / 2;

		points = {point1, point2, point3, point4};
		break;

	case SQUARE: //(0,0); (l,0); (l, l); (0, l)
		point1.x = point1.y = point2.y = point4.x = 0;
		point2.x = point3.x = point3.y = point4.y = side_length_;

		points = {point1, point2, point3, point4};
	}

	return points;
}

/** Sets matrix containing relative position of each robot
        using points of the figure to represent */
void ReynoldRulesNode::set_formation_matrix(std::vector<geometry_msgs::msg::Point> formation_points)
{
	// Set matrix initially to all 0
	formation_matrix_ = {static_cast<size_t>(NUMBER_DRONES),
	                     std::vector<geometry_msgs::msg::Point>(NUMBER_DRONES)};

	for (int i = 0; i < NUMBER_DRONES; i++) {
		for (int j = 0; j < NUMBER_DRONES; j++) {

			formation_matrix_[i][j].x = formation_points[j].x - formation_points[i].x;
			formation_matrix_[i][j].y = formation_points[j].y - formation_points[i].y;
		}
	}
}

void ReynoldRulesNode::formation_control_setup()
{
	//  Sets up everything neccesary for formation protocol if it is activated

	this->declare_parameter("formation_weight", 0.0);
	this->declare_parameter("formation_type", 1);
	this->declare_parameter("side_length", 0.0);

	this->get_parameter("formation_weight", formation_weight_);
	this->get_parameter("formation_type", formation_type_);
	this->get_parameter("side_length", side_length_);

	std::cout << "formation type: " << std::to_string(formation_type_).c_str()
	          << "\nside length: " << std::to_string(side_length_).c_str() << std::endl;

	if (formation_type_ != NONE) {
		set_formation_matrix(get_formation_points());
	}
}

void ReynoldRulesNode::rendezvous_protocol()
{
	// Implement a consensus-based rendezvous protocol. vector of a rendezvous protocol is to
	// gather the robots in a common position in space. Test the behavior for at least two fixed
	// directed communication topologies for random initial positions of the robots.
	this->paths_.clear();

	for (size_t i = 0; i < robots_.size(); i++) {
		const auto& position = robots_[i]->pose.pose.position;
		auto x               = position;
		for (size_t j = 0; j < robots_.size(); j++) {
			// implement the consensus equation
			const auto a_ij = topology_.at(i).at(j);
			const auto d    = vector_2_points(position, robots_[j]->pose.pose.position);

			// x += a_ij * d
			x = add(x, mul(a_ij, d));
		}

		// update the robot's navigation to x
		std::vector<geometry_msgs::msg::Point> path;
		path.push_back(x);
		this->paths_.emplace_back(path);
	}
}

std::vector<geometry_msgs::msg::Vector3> ReynoldRulesNode::formation_control()
{
	// If formation figure changes, create new matrix with new points
	int new_formation_type;
	this->get_parameter("formation_type", new_formation_type);

	// Formation vector will work as a rule
	std::vector<geometry_msgs::msg::Vector3> formation_vectors(NUMBER_DRONES);
	if (new_formation_type == NONE) {
		return formation_vectors;
	}

	if (new_formation_type != formation_type_) {
		formation_type_ = new_formation_type;
		set_formation_matrix(get_formation_points());
	}

	RCLCPP_INFO(get_logger(), "formation protocol type %d", formation_type_);
	for (int i = 0; i < NUMBER_DRONES; i++) {
		geometry_msgs::msg::Vector3 final_vector;

		for (int j = 0; j < NUMBER_DRONES; j++) {

			// Access matrix trasposed (Personal Implementation)
			const auto a_ij = topology_.at(j).at(i);
			geometry_msgs::msg::Vector3 vector;

			const auto& odom_i = odom(i);
			const auto& odom_j = odom(j);
			vector.x           = a_ij * (odom_j->pose.pose.position.x -
                                           odom_i->pose.pose.position.x + formation_matrix_[j][i].x);

			vector.y = a_ij * (odom_j->pose.pose.position.y -
			                   odom_i->pose.pose.position.y + formation_matrix_[j][i].y);

			final_vector.x = final_vector.x + vector.x;
			final_vector.y = final_vector.y + vector.y;
		}

		formation_vectors[i] = (final_vector);
	}

	return formation_vectors;
}

} //  namespace reynold_rules
