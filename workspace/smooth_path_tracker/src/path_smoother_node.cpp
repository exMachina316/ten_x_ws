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

#include <cmath>
#include <vector>

#include "smooth_path_tracker/path_processor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class PathSmootherNode : public rclcpp::Node {
public:
  PathSmootherNode() : Node("path_smoother_node") {
    RCLCPP_INFO(this->get_logger(), "Path Smoother node has been started.");

    this->declare_parameter<double>("desired_velocity", 0.7314);
    this->declare_parameter<double>("points_per_meter", 10.0);
    this->declare_parameter<std::vector<double>>(
        "waypoints", {0.0, 0.0, 2.0, 1.0, 4.0, -1.0, 6.0, 0.0, 8.0, 2.0});
    this->get_parameter("desired_velocity", desired_velocity_);
    this->get_parameter("points_per_meter", points_per_meter_);
    this->get_parameter("waypoints", waypoints_flat_);

    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.transient_local();
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "/smooth_path", qos_profile);
    marker_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/waypoint_markers", qos_profile);

    // Timer to publish the path periodically (or just once)
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&PathSmootherNode::generate_and_publish_path, this));
  }

private:
  void generate_and_publish_path() {
    // Define the coarse waypoints for the path
    std::vector<Point> waypoints;
    if (waypoints_flat_.size() % 2 != 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Waypoints parameter has an odd number of values. Must be "
                   "pairs of (x, y).");
      return;
    }
    for (size_t i = 0; i < waypoints_flat_.size(); i += 2) {
      waypoints.push_back({waypoints_flat_[i], waypoints_flat_[i + 1]});
    }
    visualize_waypoints(waypoints);

    // Generate smoothed time parameterized path
    nav_msgs::msg::Path smooth_path;
    smooth_path.header.frame_id = "odom";
    smooth_path.header.stamp = this->now();

    std::vector<Point> smooth_points =
        processor_.generate_smooth_path(waypoints, points_per_meter_);
    std::vector<TimedPose> timed_poses =
        processor_.generate_timed_path(smooth_points, desired_velocity_);

    if (timed_poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Generated path is empty.");
    } else {
      for (const auto &timed_pose : timed_poses) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = rclcpp::Time(0) + rclcpp::Duration::from_seconds(
                                                  timed_pose.time_offset_secs);
        pose.header.frame_id = "odom";
        pose.pose.position.x = timed_pose.x;
        pose.pose.position.y = timed_pose.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = to_quaternion(0, 0, timed_pose.yaw);
        smooth_path.poses.push_back(pose);
      }
      path_publisher_->publish(smooth_path);
      RCLCPP_INFO(this->get_logger(), "Published smooth path with %zu poses.",
                  smooth_path.poses.size());
    }

    // Stop the timer after publishing once
    timer_->cancel();
  }

  void visualize_waypoints(std::vector<Point> waypoints) {
    // Publish waypoint markers
    visualization_msgs::msg::MarkerArray marker_array;
    for (size_t i = 0; i < waypoints.size(); ++i) {
      // Create the Sphere Marker
      visualization_msgs::msg::Marker sphere_marker;
      sphere_marker.header.frame_id = "odom";
      sphere_marker.header.stamp = this->now();
      sphere_marker.ns = "waypoints_sphere";
      sphere_marker.id = i;
      sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
      sphere_marker.action = visualization_msgs::msg::Marker::ADD;
      sphere_marker.pose.position.x = waypoints[i].x;
      sphere_marker.pose.position.y = waypoints[i].y;
      sphere_marker.pose.position.z = 0.1;
      sphere_marker.pose.orientation.w = 1.0;
      sphere_marker.scale.x = 0.2;
      sphere_marker.scale.y = 0.2;
      sphere_marker.scale.z = 0.2;
      sphere_marker.color.a = 1.0;
      sphere_marker.color.r = 1.0;
      sphere_marker.color.g = 1.0;
      sphere_marker.color.b = 1.0;

      // Create the Text Marker
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = "odom";
      text_marker.header.stamp = this->now();
      text_marker.ns = "waypoints_text";
      text_marker.id = i; // Same ID to link it to the sphere
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position.x = waypoints[i].x;
      text_marker.pose.position.y = waypoints[i].y;
      text_marker.pose.position.z = 0.3; // Adjust height for text placement
      text_marker.pose.orientation.w = 1.0;
      // Scale for TEXT_VIEW_FACING only uses z for text height
      text_marker.scale.z = 0.2;
      text_marker.color.a = 1.0;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.text = "WP" + std::to_string(i);

      marker_array.markers.push_back(sphere_marker);
      marker_array.markers.push_back(text_marker);
    }
    marker_publisher_->publish(marker_array);
  }

  geometry_msgs::msg::Quaternion to_quaternion(double roll, double pitch,
                                               double yaw) {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  PathProcessor processor_;
  double desired_velocity_;
  double points_per_meter_;
  std::vector<double> waypoints_flat_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathSmootherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}