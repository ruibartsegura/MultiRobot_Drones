#include "reynold_rules_node.hpp"

ReynoldRulesNode::ReynoldRulesNode() : Node("reynold_rules_node") {
    // Inicialización de variables y configuraciones necesarias
    angular_mult_ = 1.0;
    linear_mult_ = 1.0;
    min_linear_vel_ = 0.1;
    max_linear_vel_ = 1.0;
    last_linear_vel_ = 0.0;

    separation_weight_ = 1.0;
    nav2point_weight_ = 1.0;
    obstacle_avoidance_weight_ = 1.0;
    cohesion_weight_ = 1.0;
    alignment_weight_ = 1.0;

    n_robots_ = 5; // Por ejemplo

    // Inicializar publishers y robots (esto es un ejemplo)
    for (size_t i = 0; i < n_robots_; ++i) {
        robot_names_.push_back("robot_" + std::to_string(i + 1));
        publishers_.push_back(this->create_publisher<geometry_msgs::msg::Twist>(
            "/" + robot_names_[i] + "/cmd_vel", 10));
        robots_.emplace_back(); // Agregar un odometría vacío como placeholder
    }
}

geometry_msgs::msg::Twist ReynoldRulesNode::vector2twist(const geometry_msgs::msg::Vector3 &vector, const geometry_msgs::msg::Quaternion &current_orientation) {
    geometry_msgs::msg::Twist twist;

    double current_angle = 2 * std::atan2(current_orientation.z, current_orientation.w);
    double error_angle = wrapToPi(std::atan2(vector.y, vector.x) - current_angle);

    twist.angular.z = angular_mult_ * error_angle;

    double linear_vel = (linear_mult_ * calcLength(vector) * std::exp(-std::abs(twist.angular.z)) + min_linear_vel_);
    twist.linear.x = std::min(linear_vel, max_linear_vel_);

    double linear_vel_change = std::fabs(twist.linear.x - last_linear_vel_);
    if (linear_vel_change > MAX_VEL_DIFF_FACTOR * last_linear_vel_) {
        twist.linear.x = (twist.linear.x + 2 * last_linear_vel_) / 3.0;
    }

    last_linear_vel_ = twist.linear.x;
    return twist;
}

void ReynoldRulesNode::controlCycle() {
    std::vector<std::vector<geometry_msgs::msg::Vector3>> rules = {
        separation_rule(),
        nav2point_rule(),
        obstacle_avoidance_rule(),
        alignment_rule(),
        cohesion_rule()
    };

    std::vector<double> weights = {
        separation_weight_,
        nav2point_weight_,
        obstacle_avoidance_weight_,
        cohesion_weight_,
        alignment_weight_
    };

    for (size_t i = 0; i < n_robots_; ++i) {
        geometry_msgs::msg::Vector3 total_vector;

        for (size_t j = 0; j < rules.size(); ++j) {
            geometry_msgs::msg::Vector3 new_vector;
            new_vector.x = weights[j] * rules[j][i].x;
            new_vector.y = weights[j] * rules[j][i].y;

            total_vector.x += new_vector.x;
            total_vector.y += new_vector.y;

            if (calcLength(new_vector) > max_linear_vel_) {
                break;
            }
        }

        geometry_msgs::msg::Twist vel = vector2twist(total_vector, robots_[i].pose.pose.orientation);
        publishers_[robot_names_[i]]->publish(vel);
    }
}

double ReynoldRulesNode::wrapToPi(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double ReynoldRulesNode::calcLength(const geometry_msgs::msg::Vector3 &vector) {
    return std::sqrt(vector.x * vector.x + vector.y * vector.y);
}

// Métodos para las reglas (implementar lógicas específicas)
std::vector<geometry_msgs::msg::Vector3> ReynoldRulesNode::separation_rule() {
    return std::vector<geometry_msgs::msg::Vector3>(n_robots_);
}

std::vector<geometry_msgs::msg::Vector3> ReynoldRulesNode::nav2point_rule() {
    return std::vector<geometry_msgs::msg::Vector3>(n_robots_);
}

std::vector<geometry_msgs::msg::Vector3> ReynoldRulesNode::obstacle_avoidance_rule() {
    return std::vector<geometry_msgs::msg::Vector3>(n_robots_);
}

std::vector<geometry_msgs::msg::Vector3> ReynoldRulesNode::alignment_rule() {
    return std::vector<geometry_msgs::msg::Vector3>(n_robots_);
}

std::vector<geometry_msgs::msg::Vector3> ReynoldRulesNode::cohesion_rule() {
    return std::vector<geometry_msgs::msg::Vector3>(n_robots_);
}
