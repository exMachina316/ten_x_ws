// Copyright (c) 2025 Kostubh Khandelwal
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "smooth_path_tracker/path_tracker_node.hpp"

PathTrackerNode::PathTrackerNode() : Node("path_tracker_node")
{
  RCLCPP_INFO(this->get_logger(), "Path Tracker node has been started.");

  // Parameters for the controller
  this->declare_parameter<double>("lookahead_distance", 0.5);  // meters
  this->declare_parameter<double>("max_linear_velocity", 0.2);     // m/s
  this->declare_parameter<double>("goal_tolerance", 0.1);     // meters
  this->declare_parameter<double>("yaw_error_threshold", 0.5); // radians
  this->declare_parameter<double>("prediction_horizon", 1.0);  // seconds

  this->get_parameter("lookahead_distance", lookahead_distance_);
  this->get_parameter("max_linear_velocity", max_linear_velocity_);
  this->get_parameter("goal_tolerance", goal_tolerance_);
  this->get_parameter("yaw_error_threshold", yaw_error_threshold_);
  this->get_parameter("prediction_horizon", prediction_horizon_);

  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  qos_profile.transient_local();

  path_subscription_ = this->create_subscription<nav_msgs::msg::Path>("smooth_path", qos_profile, std::bind(&PathTrackerNode::path_callback, this, std::placeholders::_1));
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&PathTrackerNode::odom_callback, this, std::placeholders::_1));

  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
  lookahead_point_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/lookahead_point", 10);
  local_plan_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/local_plan", 10);
  state_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/tracking_state_marker", 10);

  timer_ = this->create_wall_timer(50ms, std::bind(&PathTrackerNode::control_loop, this));
}

void PathTrackerNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received a new path with %zu poses.", msg->poses.size());
  current_path_ = *msg;
  target_idx_ = 0;
  if (!current_path_.poses.empty()) {
    tracking_state_ = TrackingState::APPROACHING;
    RCLCPP_INFO(this->get_logger(), "State changed to APPROACHING");
  } else {
    tracking_state_ = TrackingState::IDLE;
    RCLCPP_INFO(this->get_logger(), "Received empty path, state is IDLE");
  }
}

void PathTrackerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_odom_ = msg;
}

void PathTrackerNode::control_loop()
{
  if (tracking_state_ == TrackingState::IDLE || current_path_.poses.empty() || !last_odom_) {
    return; // No path to follow
  }

  // Current robot pose
  double robot_x = last_odom_->pose.pose.position.x;
  double robot_y = last_odom_->pose.pose.position.y;

  // Get yaw from quaternion
  tf2::Quaternion q(last_odom_->pose.pose.orientation.x,
                    last_odom_->pose.pose.orientation.y,
                    last_odom_->pose.pose.orientation.z,
                    last_odom_->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Set the target index based on the current state
  if (tracking_state_ == TrackingState::APPROACHING) {
    target_idx_ = 0;
    double first_wp_x = current_path_.poses[0].pose.position.x;
    double first_wp_y = current_path_.poses[0].pose.position.y;
    double dist_to_first_wp = std::sqrt(pow(robot_x - first_wp_x, 2) + pow(robot_y - first_wp_y, 2));

    if (dist_to_first_wp < lookahead_distance_) {
      tracking_state_ = TrackingState::TRACKING;
      RCLCPP_INFO(this->get_logger(), "State changed to TRACKING");
      // From now on, find lookahead point normally
    }
  }

  if (tracking_state_ == TrackingState::TRACKING) {
    // Find the look-ahead point on the path
    target_idx_ = find_lookahead_point(robot_x, robot_y);
  }

  // Publish lookahead point for visualization
  auto lookahead_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  lookahead_msg->header.stamp = this->get_clock()->now();
  lookahead_msg->header.frame_id = last_odom_->header.frame_id; // Use the odom frame
  lookahead_msg->point = current_path_.poses[target_idx_].pose.position;
  lookahead_point_publisher_->publish(std::move(lookahead_msg));

  // Check if the goal is reached
  double goal_x = current_path_.poses.back().pose.position.x;
  double goal_y = current_path_.poses.back().pose.position.y;
  double dist_to_goal = std::sqrt(pow(robot_x - goal_x, 2) + pow(robot_y - goal_y, 2));

  if (dist_to_goal < goal_tolerance_ && target_idx_ >= current_path_.poses.size() - 1) {
    stop_robot();
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    current_path_.poses.clear(); // Clear path to stop processing
    tracking_state_ = TrackingState::IDLE;
    return;
  }

  // Calculate control commands
  double desired_linear_velocity = max_linear_velocity_;
  get_desired_velocity(current_path_, target_idx_, desired_linear_velocity);

  double linear_velocity = 0.0;
  double angular_velocity = 0.0;
  calculate_control_commands(current_path_.poses[target_idx_].pose.position, desired_linear_velocity,
                              robot_x, robot_y, yaw,
                              linear_velocity, angular_velocity);

  // Create and publish velocity command
  auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  twist_msg->header.stamp = this->get_clock()->now();
  twist_msg->header.frame_id = "base_footprint";
  twist_msg->twist.linear.x = linear_velocity;
  twist_msg->twist.angular.z = angular_velocity;

  // Publish local plan for visualization
  publish_local_plan(robot_x, robot_y, yaw, twist_msg->twist.linear.x, twist_msg->twist.angular.z, last_odom_->header.frame_id);
  publish_state_marker(tracking_state_);

  cmd_vel_publisher_->publish(std::move(twist_msg));
}

size_t PathTrackerNode::find_lookahead_point(double robot_x, double robot_y)
{
  // Start searching from the last target index to be efficient
  for (size_t i = target_idx_; i < current_path_.poses.size(); ++i) {
    double point_x = current_path_.poses[i].pose.position.x;
    double point_y = current_path_.poses[i].pose.position.y;
    double dist = std::sqrt(pow(robot_x - point_x, 2) + pow(robot_y - point_y, 2));

    if (dist >= lookahead_distance_) {
      return i;
    }
  }
  // If no point is far enough, take the last point
  return current_path_.poses.size() - 1;
}

void PathTrackerNode::stop_robot()
{
  auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  twist_msg->header.stamp = this->get_clock()->now();
  twist_msg->header.frame_id = "base_footprint";
  twist_msg->twist.linear.x = 0.0;
  twist_msg->twist.angular.z = 0.0;
  cmd_vel_publisher_->publish(std::move(twist_msg));
  RCLCPP_INFO(this->get_logger(), "Robot stopped.");
}

void PathTrackerNode::publish_local_plan(double robot_x, double robot_y, double robot_yaw, double linear_vel, double angular_vel, const std::string &frame_id)
{
  nav_msgs::msg::Path local_plan;
  local_plan.header.stamp = this->get_clock()->now();
  local_plan.header.frame_id = frame_id;

  double dt = 0.1; // time step
  int num_points = static_cast<int>(prediction_horizon_ / dt);

  double current_x = robot_x;
  double current_y = robot_y;
  double current_yaw = robot_yaw;

  for (int i = 0; i < num_points; ++i) {
    current_x += linear_vel * cos(current_yaw) * dt;
    current_y += linear_vel * sin(current_yaw) * dt;
    current_yaw += angular_vel * dt;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = rclcpp::Time(local_plan.header.stamp) + rclcpp::Duration::from_seconds(i * dt);
    pose.header.frame_id = frame_id;
    pose.pose.position.x = current_x;
    pose.pose.position.y = current_y;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    local_plan.poses.push_back(pose);
  }
  local_plan_publisher_->publish(local_plan);
}

void PathTrackerNode::publish_state_marker(TrackingState state)
{
  auto marker_msg = std::make_unique<visualization_msgs::msg::Marker>();
  marker_msg->header.stamp = this->get_clock()->now();
  marker_msg->header.frame_id = "base_footprint";
  marker_msg->ns = "tracking_state";
  marker_msg->id = 0;
  marker_msg->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker_msg->action = visualization_msgs::msg::Marker::ADD;
  marker_msg->pose.position.x = 0.0; // Position in front of the robot
  marker_msg->pose.position.y = 0.0;
  marker_msg->pose.position.z = 0.5; // Position above the base
  marker_msg->pose.orientation.w = 1.0;
  marker_msg->scale.z = 0.2; // Text height
  marker_msg->color.a = 1.0;
  marker_msg->color.r = 0.0;
  marker_msg->color.g = 1.0;
  marker_msg->color.b = 0.0;
  marker_msg->text = trackingStateToString(state);
  marker_msg->lifetime = rclcpp::Duration::from_seconds(1.0);

  state_marker_publisher_->publish(std::move(marker_msg));
}

void PathTrackerNode::get_desired_velocity(const nav_msgs::msg::Path &path, size_t target_idx, double &desired_linear_velocity)
{
  if (tracking_state_ == TrackingState::TRACKING &&
      path.poses.size() > 1 && target_idx > 0) {
    rclcpp::Time t1(path.poses[target_idx - 1].header.stamp);
    rclcpp::Time t2(path.poses[target_idx].header.stamp);
    double dt = (t2 - t1).seconds();

    if (dt > 1e-6) { // Avoid division by zero
      double p1_x = path.poses[target_idx - 1].pose.position.x;
      double p1_y = path.poses[target_idx - 1].pose.position.y;
      double p2_x = path.poses[target_idx].pose.position.x;
      double p2_y = path.poses[target_idx].pose.position.y;
      double dist = std::sqrt(pow(p2_x - p1_x, 2) + pow(p2_y - p1_y, 2));
      desired_linear_velocity = dist / dt;
    }
  }
}

void PathTrackerNode::calculate_control_commands(const geometry_msgs::msg::Point &lookahead_point, double desired_linear_velocity,
                                double robot_x, double robot_y, double robot_yaw,
                                double &linear_velocity, double &angular_velocity)
{
  double target_x = lookahead_point.x;
  double target_y = lookahead_point.y;

  double alpha = atan2(target_y - robot_y, target_x - robot_x) - robot_yaw;
  alpha = atan2(sin(alpha), cos(alpha)); // Normalize alpha to [-PI, PI]

  if (std::abs(alpha) > yaw_error_threshold_) {
    linear_velocity = 0.0;
    angular_velocity = std::copysign(max_linear_velocity_, alpha);
    return;
  }

  double curvature = 2.0 * sin(alpha) / lookahead_distance_;

  linear_velocity = desired_linear_velocity;
  angular_velocity = desired_linear_velocity * curvature;
}

std::string PathTrackerNode::trackingStateToString(TrackingState state)
{
  switch (state) {
  case TrackingState::IDLE:
    return "IDLE";
  case TrackingState::APPROACHING:
    return "APPROACHING";
  case TrackingState::TRACKING:
    return "TRACKING";
  default:
    return "UNKNOWN";
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathTrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}