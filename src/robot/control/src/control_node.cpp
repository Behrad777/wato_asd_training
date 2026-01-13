#include "control_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <optional>

ControlNode::ControlNode() : Node("pure_pursuit_controller") {
    // Initialize parameters
    lookahead_distance_ = 1.0;  // Lookahead distance (meters)
    goal_tolerance_ = 0.1;     // Distance to consider the goal reached (meters)
    linear_speed_ = 1;       // Constant forward speed (m/s)

    // Subscribers and Publishers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; }); //lalready implemented here as a lambda

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer
    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { controlLoop(); });
}
void ControlNode::controlLoop() {
    if (!current_path_ || !robot_odom_) {
        return;
    }

    //if path is empty stop the bot
    if (current_path_->poses.empty()) {
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }

    const auto& final_pose = current_path_->poses.back().pose.position;
    const auto& cur_pos = robot_odom_->pose.pose.position;
    
    if (computeDistance(cur_pos, final_pose) < goal_tolerance_) {
        //set velocity to 0, for some reason planner isnt stopping bot
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Goal reached! Stopping.");
        return;
    }

    //stop based on lookahead points
    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }

    auto cmd_vel = computeVelocity(*lookahead_point);
    cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
        return std::nullopt;
    }

    const auto& cur_pos = robot_odom_->pose.pose.position;

    //find closest position on the path 
    size_t closest_idx = 0; 
    double min_dist = std::numeric_limits<double>::infinity();

    for(size_t i{0}; i< current_path_->poses.size(); ++i){ //iterate over the path indexes 
        const auto& path_position = current_path_->poses[i].pose.position;
        double distance_to_position = computeDistance(cur_pos, path_position);

        if(distance_to_position < min_dist){ //keep a track of the current min distance over all the path
        min_dist = distance_to_position;
        closest_idx = i;
        }

    }

    //now start on the closest point and find the target
    for (size_t i = closest_idx; i < current_path_->poses.size(); ++i) {
        const auto& path_position = current_path_->poses[i].pose.position;
        double distance_to_position = computeDistance(cur_pos, path_position);
        
        if(computeDistance(cur_pos, path_position) >=lookahead_distance_){ //first one we find thats at the lookahead distance 
            return current_path_->poses[i];
        }

    }    

    //if there are no other points at the lookahead distance, go straight to the goal 
    return current_path_->poses.back();
}

//velocity to reach the next target, not current velocity
geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
    //need the next state as well as the current state of positions to compute velocity we need 
    geometry_msgs::msg::Twist cmd_vel;

    if(!robot_odom_) return cmd_vel; //if we didnt get an update on the odometry 

    const auto& cur_pos = robot_odom_->pose.pose.position;
    const auto& quaternion = robot_odom_->pose.pose.orientation; //comes as a quaternion object

    double yaw = extractYaw(quaternion);

    double dx = target.pose.position.x - cur_pos.x;
    double dy = target.pose.position.y - cur_pos.y;
    //z isnt gonna matter, not driving up walls lol

    //2D rotation matrix into robot's frame
    double x_robot = std::cos(yaw) * dx + std::sin(yaw) * dy;
    double y_robot = -std::sin(yaw) * dx + std::cos(yaw) * dy;

    double heading_error = std::atan2(y_robot, x_robot);

    const double enter_rotate = M_PI / 2.0;   // 90 deg
    const double exit_rotate = M_PI / 6.0;    // 30 deg
    const double max_turn_rate = 1.0;
    const double turn_gain = 1.5;


    //make sure we fully rotate instead of oscillating
    if (!rotating_in_place_) {
        if (x_robot < 0.0 || std::fabs(heading_error) > enter_rotate) {
            rotating_in_place_ = true;
        }
    } else {
        if (x_robot > 0.0 && std::fabs(heading_error) < exit_rotate) {
            rotating_in_place_ = false;
        }
    }

    if (rotating_in_place_) {
        cmd_vel.linear.x = 0.0;
        double angular = turn_gain * heading_error;
        if (angular > max_turn_rate) angular = max_turn_rate;
        if (angular < -max_turn_rate) angular = -max_turn_rate;
        cmd_vel.angular.z = angular;
        return cmd_vel;
    }

    double L = std::max(lookahead_distance_, 0.001); //lookahead distance is at least 0.001m 

    //how sharply robot should turn, pure pursuit κ = 2*y / L^2
    double kappa = (2*y_robot) / (L*L);

    //never changed the linear speed, cuz pure pursuit
    cmd_vel.linear.x = linear_speed_;
    //angular speed can change, and we calculated kappa, dw
    cmd_vel.angular.z = linear_speed_ * kappa;

    return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2) + std::pow(b.z - a.z, 2)); //classic formula
}

//
double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    //quaternion to euler yaw
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
}

// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
