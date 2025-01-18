#ifndef REYNOLD_RULES__REYNOLDRULESNODE_HPP_
#define REYNOLD_RULES__REYNOLDRULESNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <geometry_msgs/msg/quaternion.hpp>

#include <optional>
#include <queue>
#include <set>
#include <vector>
#include <array>

namespace reynold_rules {

/// determines the source for the navigation rule
enum class NavigationMethod {
	RosParam,  // fixed point passed via rosparam
	Rendezvous, // update point via rendezvous protocol
};

class ReynoldRulesNode : public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(ReynoldRulesNode)

	ReynoldRulesNode();
	void control_cycle();

	std::vector<geometry_msgs::msg::Vector3> separation_rule();
	std::vector<geometry_msgs::msg::Vector3> aligment_rule();
	std::vector<geometry_msgs::msg::Vector3> cohesion_rule();
	std::vector<geometry_msgs::msg::Vector3> nav_2_point_rule();
	std::vector<geometry_msgs::msg::Vector3> avoidance_rule();

	/// @brief execute the rendezvous_protocol and update each robot's path in paths_
	void rendezvous_protocol();
  	void formation_control_setup();
  	std::vector<geometry_msgs::msg::Vector3> formation_control();

private:
	bool READY = false;

	static const int NUMBER_DRONES{4};
	double MAX_LIN_VEL{1};
	double MIN_LIN_VEL{0.0};
	double DIST_THRESHOLD{1};

	double separation_weight_{1.0};
	double cohesion_weight_{0.0};
	double alignment_weight_{0.0};
	double nav2point_weight_{0.0};
	double obstacle_avoidance_weight_{0.0};
	double formation_weight_ {0.0};

	// Formation Control
	int formation_type_;
	static const int NONE = 1;
	static const int LINE = 2;
	static const int TRIANGLE = 3;
	static const int SQUARE = 4;

	std::vector<std::vector<geometry_msgs::msg::Point>> formation_matrix_;
	std::vector<geometry_msgs::msg::Point> get_formation_points(int, float);
	void set_formation_matrix(std::vector<geometry_msgs::msg::Point>);


	std::vector<nav_msgs::msg::Odometry::SharedPtr> robots_;

	// Pub and subs vectors
	std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> publishers_;
	std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subscribers_;
	static constexpr double ANGULAR_STEPSIZE {2 * (M_PI / 180.0)};

	// Map
	nav_msgs::msg::OccupancyGrid::SharedPtr map_;

	// Separation
	double view_range_{0.4};
	//double get_distance(geometry_msgs::msg::Point pos1, geometry_msgs::msg::Point pos2);
	geometry_msgs::msg::Vector3 calc_sep_vector(geometry_msgs::msg::Point position, int num);

	// Alignment
	geometry_msgs::msg::Vector3 calc_average_velocity();

	// Cohesion
	geometry_msgs::msg::Point calc_average_pos(std::vector<nav_msgs::msg::Odometry> positions);
	geometry_msgs::msg::Vector3 calc_cohesion_vector(geometry_msgs::msg::Point robot_pos);

	// Obstacle avoidance
	double view_angle_{0.4};
	double view_split_{0.0};
	bool map_lookup(geometry_msgs::msg::Point& pos);
	std::vector<geometry_msgs::msg::Point> find_obstacles(const geometry_msgs::msg::Pose robot_pose);
	geometry_msgs::msg::Vector3 sum_weighted_repellent_vectors(int robot_index);

	// Nav_2_Point
	void recalculatePath();
	std::vector<geometry_msgs::msg::Point> findPathThroughWaypoints(
			const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& target);

	std::vector<geometry_msgs::msg::Point> findNeighbors(
			const std::vector<geometry_msgs::msg::Point>& waypoints,
			const geometry_msgs::msg::Point& currentWp, int step = 2);

	bool isPathClear(const std::pair<int, int>& start, const std::pair<int, int>& end);

	geometry_msgs::msg::Vector3 vector_2_points(geometry_msgs::msg::Point point1,
												geometry_msgs::msg::Point point2,
												std::optional<double> max_length = std::nullopt);

	NavigationMethod navigationMethod_{NavigationMethod::RosParam};
	geometry_msgs::msg::Point target_point_;
	geometry_msgs::msg::Point prev_point;
	std::vector<geometry_msgs::msg::Point> waypoints_;
	std::vector<geometry_msgs::msg::Point> path_;
	std::vector<std::vector<geometry_msgs::msg::Point>> paths_;

	double calc_length(const geometry_msgs::msg::Vector3 &vector);

	double angular_mult_{2.0};
	double linear_mult_{1.0};
	double last_linear_vel_ = 0;
	double MAX_VEL_DIFF_FACTOR {0.2};

	std::vector<std::vector<double>> topology_;

	// Subscribers
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr data);
	void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr data);
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

	void checkPathsBetweenWaypoints();

	rclcpp::TimerBase::SharedPtr timer_;
};

} //  namespace reynold_rules

#endif // REYNOLD_RULES__REYNOLDRULESNODE_HPP_
