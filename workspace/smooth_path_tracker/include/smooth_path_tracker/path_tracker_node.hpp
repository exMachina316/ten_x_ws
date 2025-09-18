#ifndef SMOOTH_PATH_TRACKER__PATH_TRACKER_NODE_HPP_
#define SMOOTH_PATH_TRACKER__PATH_TRACKER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

/**
 * @brief Enum for the tracking state of the robot.
 */
enum class TrackingState { IDLE, APPROACHING, TRACKING };

using namespace std::chrono_literals;

/**
 * @class PathTrackerNode
 * @brief A ROS2 node for tracking a given path using a pure pursuit algorithm.
 */
class PathTrackerNode : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Path Tracker Node object
   */
  PathTrackerNode();

  /**
   * @brief Callback for the path subscription.
   * @param msg The received path message.
   */
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

  /**
   * @brief Callback for the odometry subscription.
   * @param msg The received odometry message.
   */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Main control loop for the path tracker.
   */
  void control_loop();

  /**
   * @brief Find the lookahead point on the current path.
   * @param robot_x Current x position of the robot.
   * @param robot_y Current y position of the robot.
   * @return size_t Index of the lookahead point in the path.
   */
  size_t find_lookahead_point(double robot_x, double robot_y);

  /**
   * @brief Stops the robot by publishing a zero velocity command.
   */
  void stop_robot();

  /**
   * @brief Publish the local plan for visualization.
   * @param robot_x Current x position of the robot.
   * @param robot_y Current y position of the robot.
   * @param robot_yaw Current yaw of the robot.
   * @param linear_vel Current linear velocity of the robot.
   * @param angular_vel Current angular velocity of the robot.
   * @param frame_id The frame ID for the local plan.
   */
  void publish_local_plan(double robot_x, double robot_y, double robot_yaw, double linear_vel, double angular_vel, const std::string &frame_id);

  /**
   * @brief Publish the current tracking state as a marker for visualization.
   * @param state The current tracking state.
   */
  void publish_state_marker(TrackingState state);

  /**
   * @brief Get the desired linear velocity based on the path.
   * @param path The current path.
   * @param target_idx The index of the target point.
   * @param[out] desired_linear_velocity The calculated desired linear velocity.
   */
  void get_desired_velocity(const nav_msgs::msg::Path &path, size_t target_idx, double &desired_linear_velocity);

  /**
   * @brief Calculate the control commands (linear and angular velocity).
   * @param lookahead_point The lookahead point.
   * @param desired_linear_velocity The desired linear velocity.
   * @param robot_x Current x position of the robot.
   * @param robot_y Current y position of the robot.
   * @param robot_yaw Current yaw of the robot.
   * @param[out] linear_velocity The calculated linear velocity.
   * @param[out] angular_velocity The calculated angular velocity.
   */
  void calculate_control_commands(const geometry_msgs::msg::Point &lookahead_point, double desired_linear_velocity,
                                  double robot_x, double robot_y, double robot_yaw,
                                  double &linear_velocity, double &angular_velocity);

  /**
   * @brief Convert a TrackingState enum to its string representation.
   * @param state The tracking state.
   * @return std::string The string representation of the state.
   */
  std::string trackingStateToString(TrackingState state);

  // State variables
  size_t target_idx_ = 0; ///< Index of the current target point in the path.
  TrackingState tracking_state_ = TrackingState::IDLE; ///< Current tracking state.

  // Configs
  double lookahead_distance_;   ///< Lookahead distance for the pure pursuit controller.
  double max_linear_velocity_;  ///< Maximum linear velocity of the robot.
  double goal_tolerance_;       ///< Tolerance for reaching the goal.
  double yaw_error_threshold_;  ///< Yaw error threshold to trigger in-place rotation.
  double prediction_horizon_;   ///< Prediction horizon for local plan visualization.

  // Callback data holders
  nav_msgs::msg::Path current_path_; ///< The current path to follow.
  nav_msgs::msg::Odometry::SharedPtr last_odom_; ///< The last received odometry message.

  // Timer
  rclcpp::TimerBase::SharedPtr timer_; ///< Timer for the main control loop.

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_; ///< Subscription to the path topic.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_; ///< Subscription to the odometry topic.

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_plan_publisher_; ///< Publisher for the local plan.
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_; ///< Publisher for the velocity command.
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_point_publisher_; ///< Publisher for the lookahead point.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr state_marker_publisher_; ///< Publisher for the state marker.
};

#endif  // SMOOTH_PATH_TRACKER__PATH_TRACKER_NODE_HPP_
