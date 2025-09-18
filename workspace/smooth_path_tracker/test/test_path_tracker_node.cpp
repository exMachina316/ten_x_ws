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

#include <math.h>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "smooth_path_tracker/path_tracker_node.hpp"


class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class Controller : public PathTrackerNode
{
    public:
    Controller() : PathTrackerNode() {}

    void configure(double lookahead_distance, double goal_tolerance, double max_linear_velocity, double yaw_error_threshold) {
      lookahead_distance_ = lookahead_distance;
      goal_tolerance_ = goal_tolerance;
      max_linear_velocity_ = max_linear_velocity;
      yaw_error_threshold_ = yaw_error_threshold;
    }

    void set_state(TrackingState new_state) {
      tracking_state_ = new_state;
    }

    TrackingState get_state() {
      return tracking_state_;
    }
};

TEST(PathTrackerNodeTest, FindLookaheadPointAPI)
{
  auto controller = std::make_shared<Controller>();

  // Create a simple path
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.position.x = i * 0.5; // Points at (0,0), (0.5,0), ..., (5.0,0)
    pose.pose.position.y = 0.0;
    path.poses.push_back(pose);
  }
  controller->path_callback(std::make_shared<nav_msgs::msg::Path>(path));

  double robot_x, robot_y;
  double lookahead_distance, goal_tolerance, max_linear_velocity, yaw_error_threshold;
  size_t lookahead_idx;

  // Test case 1: Robot at (0,0)
  robot_x = 0.0;
  robot_y = 0.0;
  lookahead_distance = 1.0;
  goal_tolerance = 0.1;
  max_linear_velocity = 0.2;
  yaw_error_threshold = 0.5;
  controller->configure(lookahead_distance, goal_tolerance, max_linear_velocity, yaw_error_threshold);

  lookahead_idx = controller->find_lookahead_point(robot_x, robot_y);
  EXPECT_EQ(lookahead_idx, 2); // Expect index 2 (1.0m ahead)

  // Test case 2: Robot at (1.0,0)
  robot_x = 1.0;
  robot_y = 0.0;
  lookahead_distance = 1.5;
  goal_tolerance = 0.1;
  max_linear_velocity = 0.2;
  yaw_error_threshold = 0.5;
  controller->configure(lookahead_distance, goal_tolerance, max_linear_velocity, yaw_error_threshold);

  lookahead_idx = controller->find_lookahead_point(robot_x, robot_y);
  EXPECT_EQ(lookahead_idx, 5); // Expect index 5 (2.5m ahead)
}

TEST(PathTrackerNodeTest, StateTransitions)
{
  auto controller = std::make_shared<Controller>();
  
  double lookahead_distance, goal_tolerance, max_linear_velocity, yaw_error_threshold;

  lookahead_distance = 0.5;
  goal_tolerance = 0.1;
  max_linear_velocity = 0.2;
  yaw_error_threshold = 0.5;
  controller->configure(lookahead_distance, goal_tolerance, max_linear_velocity, yaw_error_threshold);

  // Initial state should be IDLE
  EXPECT_EQ(controller->get_state(), TrackingState::IDLE);

  // Empty path should keep state as IDLE
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = "odom";
  controller->path_callback(std::make_shared<nav_msgs::msg::Path>(empty_path));
  EXPECT_EQ(controller->get_state(), TrackingState::IDLE);

  // A valid path should change state to APPROACHING
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 0.0;
  path.poses.push_back(pose);
  controller->path_callback(std::make_shared<nav_msgs::msg::Path>(path));
  EXPECT_EQ(controller->get_state(), TrackingState::APPROACHING);

  // Simulate robot moving close to the first waypoint
  auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  odom_msg->header.frame_id = "odom";
  odom_msg->pose.pose.position.x = 0.8; // Close to the first waypoint
  odom_msg->pose.pose.position.y = 0.0;
  controller->odom_callback(odom_msg);
  controller->configure(lookahead_distance, goal_tolerance, max_linear_velocity, yaw_error_threshold);
  controller->control_loop();
  EXPECT_EQ(controller->get_state(), TrackingState::TRACKING);

  // Simulate reaching the goal
  odom_msg->pose.pose.position.x = 1.0; // At the goal
  odom_msg->pose.pose.position.y = 0.0;
  controller->odom_callback(odom_msg);
  controller->control_loop();
  EXPECT_EQ(controller->get_state(), TrackingState::IDLE);
}

TEST(PathTrackerNodeTest, StopRobotTest)
{
  auto controller = std::make_shared<Controller>();
  bool message_received = false;
  geometry_msgs::msg::TwistStamped::SharedPtr received_msg;

  auto cmd_vel_sub = controller->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel", 10,
    [&](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      message_received = true;
      received_msg = msg;
    });

  controller->stop_robot();

  // Spin for a short time to allow message to be processed
  rclcpp::spin_some(controller);
  for(int i = 0; i < 10 && !message_received; ++i) {
    rclcpp::spin_some(controller);
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_TRUE(message_received);
  EXPECT_EQ(received_msg->twist.linear.x, 0.0);
  EXPECT_EQ(received_msg->twist.angular.z, 0.0);
}

TEST(PathTrackerNodeTest, CalculateControlCommands)
{
  auto controller = std::make_shared<Controller>();

  double lookahead_distance, goal_tolerance, max_linear_velocity, yaw_error_threshold;

  lookahead_distance = 1.0;
  goal_tolerance = 0.1;
  max_linear_velocity = 0.2;
  yaw_error_threshold = 0.5;
  controller->configure(lookahead_distance, goal_tolerance, max_linear_velocity, yaw_error_threshold);

  geometry_msgs::msg::Point lookahead_point;
  double linear_velocity, angular_velocity;

  // Test case 1: Robot at origin, pointing along X, lookahead point on X axis
  lookahead_point.x = 1.0;
  lookahead_point.y = 0.0;
  controller->calculate_control_commands(lookahead_point, 0.5, 0.0, 0.0, 0.0, linear_velocity, angular_velocity);
  EXPECT_NEAR(linear_velocity, 0.5, 1e-6);
  EXPECT_NEAR(angular_velocity, 0.0, 1e-6);

  // Test case 2: Robot at origin, pointing along X, lookahead point at (1,0.5)
  lookahead_point.x = 1.0;
  lookahead_point.y = 0.5;
  controller->calculate_control_commands(lookahead_point, 0.5, 0.0, 0.0, 0.0, linear_velocity, angular_velocity);
  EXPECT_NEAR(linear_velocity, 0.5, 1e-6);
  EXPECT_GT(angular_velocity, 0.0); // Should turn left

  // Test case 3: Large yaw error, should rotate in place
  lookahead_point.x = 0.0;
  lookahead_point.y = 1.0;
  controller->calculate_control_commands(lookahead_point, 0.5, 0.0, 0.0, 0.0, linear_velocity, angular_velocity);
  EXPECT_EQ(linear_velocity, 0.0);
  EXPECT_GT(angular_velocity, 0.0);
}

TEST(PathTrackerNodeTest, GetDesiredVelocity)
{
  auto controller = std::make_shared<Controller>();
  controller->set_state(TrackingState::TRACKING);

  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  
  geometry_msgs::msg::PoseStamped pose1;
  pose1.header.stamp = rclcpp::Time(0, 0);
  pose1.pose.position.x = 0.0;
  pose1.pose.position.y = 0.0;
  path.poses.push_back(pose1);

  geometry_msgs::msg::PoseStamped pose2;
  pose2.header.stamp = rclcpp::Time(1, 0); // 1 second later
  pose2.pose.position.x = 0.5; // 0.5m away
  pose2.pose.position.y = 0.0;
  path.poses.push_back(pose2);

  double desired_linear_velocity = 0.0;
  controller->get_desired_velocity(path, 1, desired_linear_velocity);
  EXPECT_NEAR(desired_linear_velocity, 0.5, 1e-6);
}

TEST(PathTrackerNodeTest, EmptyPath)
{
  auto controller = std::make_shared<Controller>();
  
  // Initially in IDLE state
  EXPECT_EQ(controller->get_state(), TrackingState::IDLE);

  // Send a valid path first to change state
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 1.0;
  path.poses.push_back(pose);
  controller->path_callback(std::make_shared<nav_msgs::msg::Path>(path));
  EXPECT_EQ(controller->get_state(), TrackingState::APPROACHING);

  // Now send an empty path
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = "odom";
  controller->path_callback(std::make_shared<nav_msgs::msg::Path>(empty_path));
  EXPECT_EQ(controller->get_state(), TrackingState::IDLE);

  // Ensure control loop doesn't do anything
  auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  controller->odom_callback(odom_msg);
  // This should not crash or change state
  controller->control_loop(); 
  EXPECT_EQ(controller->get_state(), TrackingState::IDLE);
}

TEST(PathTrackerNodeTest, TrackingStateToString)
{
    auto controller = std::make_shared<Controller>();
    EXPECT_EQ(controller->trackingStateToString(TrackingState::IDLE), "IDLE");
    EXPECT_EQ(controller->trackingStateToString(TrackingState::APPROACHING), "APPROACHING");
    EXPECT_EQ(controller->trackingStateToString(TrackingState::TRACKING), "TRACKING");
}

TEST(PathTrackerNodeTest, GoalTolerance)
{
  auto controller = std::make_shared<Controller>();
  
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 0.0;
  path.poses.push_back(pose);
  controller->path_callback(std::make_shared<nav_msgs::msg::Path>(path));
  
  auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  odom_msg->header.frame_id = "odom";

  double lookahead_distance, goal_tolerance, max_linear_velocity, yaw_error_threshold;

  // Case 1: Outside goal tolerance
  lookahead_distance = 0.5;
  goal_tolerance = 0.2;
  max_linear_velocity = 0.2;
  yaw_error_threshold = 0.5;
  controller->configure(lookahead_distance, goal_tolerance, max_linear_velocity, yaw_error_threshold);

  controller->set_state(TrackingState::TRACKING);
  odom_msg->pose.pose.position.x = 0.7; // 0.3m away from goal
  controller->odom_callback(odom_msg);
  controller->control_loop();
  EXPECT_NE(controller->get_state(), TrackingState::IDLE);

  // Case 2: Inside goal tolerance
  lookahead_distance = 0.5;
  goal_tolerance = 0.4;
  max_linear_velocity = 0.2;
  yaw_error_threshold = 0.5;
  controller->configure(lookahead_distance, goal_tolerance, max_linear_velocity, yaw_error_threshold);

  controller->set_state(TrackingState::TRACKING);
  odom_msg->pose.pose.position.x = 0.7; // 0.3m away from goal
  controller->odom_callback(odom_msg);
  controller->control_loop();
  EXPECT_EQ(controller->get_state(), TrackingState::IDLE);
}

TEST(PathTrackerNodeTest, MaxLinearVelocity)
{
  auto controller = std::make_shared<Controller>();

  double lookahead_distance, goal_tolerance, max_linear_velocity, yaw_error_threshold;

  lookahead_distance = 1.0;
  goal_tolerance = 0.1;
  max_linear_velocity = 1.23;
  yaw_error_threshold = 0.5;
  controller->configure(lookahead_distance, goal_tolerance, max_linear_velocity, yaw_error_threshold);

  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 2.0;
  path.poses.push_back(pose);
  controller->path_callback(std::make_shared<nav_msgs::msg::Path>(path));
  controller->set_state(TrackingState::TRACKING);

  auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  odom_msg->header.frame_id = "odom";
  odom_msg->pose.pose.position.x = 0.0;
  controller->odom_callback(odom_msg);

  bool message_received = false;
  geometry_msgs::msg::TwistStamped::SharedPtr received_msg;
  auto cmd_vel_sub = controller->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel", 10,
    [&](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      message_received = true;
      received_msg = msg;
    });

  controller->control_loop();
  rclcpp::spin_some(controller);
  for(int i = 0; i < 10 && !message_received; ++i) {
    rclcpp::spin_some(controller);
    std::this_thread::sleep_for(10ms);
  }
  
  ASSERT_TRUE(message_received);
  EXPECT_NEAR(received_msg->twist.linear.x, 1.23, 1e-6);
}

TEST(PathTrackerNodeTest, YawErrorThreshold)
{
  auto controller = std::make_shared<Controller>();
  
  double lookahead_distance, goal_tolerance, max_linear_velocity, yaw_error_threshold;

  lookahead_distance = 1.0;
  goal_tolerance = 0.1;
  max_linear_velocity = 0.5;
  yaw_error_threshold = 0.5;
  controller->configure(lookahead_distance, goal_tolerance, max_linear_velocity, yaw_error_threshold);

  geometry_msgs::msg::Point lookahead_point;
  double linear_velocity, angular_velocity;

  // Case 1: Yaw error below threshold
  lookahead_point.x = cos(0.4);
  lookahead_point.y = sin(0.4);
  controller->calculate_control_commands(lookahead_point, 0.5, 0.0, 0.0, 0.0, linear_velocity, angular_velocity);
  EXPECT_NE(linear_velocity, 0.0);

  // Case 2: Yaw error above threshold
  controller->configure(1.0, 0.1, 0.5, 0.5);
  lookahead_point.x = cos(0.6);
  lookahead_point.y = sin(0.6);
  controller->calculate_control_commands(lookahead_point, 0.5, 0.0, 0.0, 0.0, linear_velocity, angular_velocity);
  EXPECT_EQ(linear_velocity, 0.0);
  EXPECT_GT(angular_velocity, 0.0);
}