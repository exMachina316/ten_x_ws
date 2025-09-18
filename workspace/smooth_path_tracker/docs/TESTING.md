\# Test Design

This document outlines the test design for the `smooth_path_tracker` package, including the `path_tracker_node` and `path_processor` components. The testing strategy focuses on unit tests to ensure the correctness and robustness of the core functionalities.

## 1. Test Strategy

The primary testing approach is unit testing using the **Google Test (gtest)** framework. This allows for isolated testing of individual functions and behaviors. For ROS nodes like `PathTrackerNode`, a mock class derived from the node is used to facilitate testing by providing access to internal states and protected members.

The tests are designed to cover the following key areas:
- **State Machine Logic:** Ensuring correct transitions between states.
- **Core Algorithms:** Validating the underlying logic, such as pure pursuit, path smoothing, and command generation.
- **Parameter Handling:** Verifying that components' behavior changes correctly based on configuration.
- **Edge Cases:** Testing the response to non-nominal inputs, such as empty paths or invalid data.
- **Helper Functions:** Ensuring the correctness of utility functions.

## 2. Test Environment

- **Framework:** ROS2 Foxy
- **Testing Library:** `gtest`
- **Dependencies:** `rclcpp`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`

## 3. Path Tracker Node Tests

This section details the test cases for the `path_tracker_node`.

### 3.1. `FindLookaheadPointAPI`
- **Objective:** To verify that the `find_lookahead_point` function correctly identifies the target point on the path based on the robot's position and the lookahead distance.

| Test Scenario | Inputs | Expected Outcome | Assertion(s) |
| :--- | :--- | :--- | :--- |
| Robot at origin | `robot_x=0.0`, `robot_y=0.0`, `lookahead_distance=1.0` | `lookahead_idx = 2` | `EXPECT_EQ` |
| Robot along the path | `robot_x=1.0`, `robot_y=0.0`, `lookahead_distance=1.5` | `lookahead_idx = 5` | `EXPECT_EQ` |

### 3.2. `StateTransitions`
- **Objective:** To ensure the node transitions correctly between `IDLE`, `APPROACHING`, and `TRACKING` states.

| Test Scenario | Action | Expected State | Assertion(s) |
| :--- | :--- | :--- | :--- |
| Initial State | Node is initialized | `IDLE` | `EXPECT_EQ` |
| Empty Path Received | `path_callback` with an empty path | `IDLE` | `EXPECT_EQ` |
| Valid Path Received | `path_callback` with a valid path | `APPROACHING` | `EXPECT_EQ` |
| Approaching Waypoint | Robot moves close to the first waypoint | `TRACKING` | `EXPECT_EQ` |
| Goal Reached | Robot moves to the goal position | `IDLE` | `EXPECT_EQ` |

### 3.3. `StopRobotTest`
- **Objective:** To confirm that the `stop_robot` function publishes a zero-velocity command.

| Test Scenario | Action | Expected Outcome | Assertion(s) |
| :--- | :--- | :--- | :--- |
| Stop Command | `stop_robot()` is called | A `TwistStamped` message with zero velocities is published | `ASSERT_TRUE`, `EXPECT_EQ` |

### 3.4. `CalculateControlCommands`
- **Objective:** To validate the logic for calculating linear and angular velocity commands.

| Test Scenario | Inputs | Expected Outcome | Assertion(s) |
| :--- | :--- | :--- | :--- |
| Straight Motion | Lookahead point is directly ahead | `linear_velocity > 0`, `angular_velocity` is near zero | `EXPECT_NEAR` |
| Left Turn | Lookahead point is to the left | `linear_velocity > 0`, `angular_velocity > 0` | `EXPECT_NEAR`, `EXPECT_GT` |
| In-place Rotation | Yaw error exceeds `yaw_error_threshold_` | `linear_velocity = 0`, `angular_velocity > 0` | `EXPECT_EQ`, `EXPECT_GT` |

### 3.5. `GetDesiredVelocity`
- **Objective:** To verify the calculation of the desired linear velocity based on path timestamps.

| Test Scenario | Inputs | Expected Outcome | Assertion(s) |
| :--- | :--- | :--- | :--- |
| Velocity from Path | Path with two points 0.5m apart with a 1s timestamp difference | `desired_linear_velocity` is near 0.5 m/s | `EXPECT_NEAR` |

### 3.6. `EmptyPath`
- **Objective:** To ensure the node handles receiving an empty path gracefully.

| Test Scenario | Action | Expected Outcome | Assertion(s) |
| :--- | :--- | :--- | :--- |
| Receive Empty Path | `path_callback` with an empty path while in `APPROACHING` state | State transitions to `IDLE` and no crash occurs | `EXPECT_EQ` |

### 3.7. `TrackingStateToString`
- **Objective:** To test the correctness of the `trackingStateToString` utility function.

| Test Scenario | Input State | Expected String | Assertion(s) |
| :--- | :--- | :--- | :--- |
| IDLE state | `TrackingState::IDLE` | "IDLE" | `EXPECT_EQ` |
| APPROACHING state | `TrackingState::APPROACHING` | "APPROACHING" | `EXPECT_EQ` |
| TRACKING state | `TrackingState::TRACKING` | "TRACKING" | `EXPECT_EQ` |

### 3.8. Parameter Tests (`GoalTolerance`, `MaxLinearVelocity`, `YawErrorThreshold`)
- **Objective:** To verify that the controller's behavior correctly reflects its configuration parameters.

| Parameter | Test Scenario | Inputs | Expected Outcome | Assertion(s) |
| :--- | :--- | :--- | :--- | :--- |
| `goal_tolerance` | Robot is outside the tolerance | `dist_to_goal > goal_tolerance` | State is not `IDLE` | `EXPECT_NE` |
| `goal_tolerance` | Robot is inside the tolerance | `dist_to_goal < goal_tolerance` | State becomes `IDLE` | `EXPECT_EQ` |
| `max_linear_velocity` | Path has no timestamps | `max_linear_velocity` is set to `1.23` | Published linear velocity is `1.23` | `EXPECT_NEAR` |
| `yaw_error_threshold` | Yaw error is below threshold | `alpha < yaw_error_threshold` | `linear_velocity > 0` | `EXPECT_NE` |
| `yaw_error_threshold` | Yaw error is above threshold | `alpha > yaw_error_threshold` | `linear_velocity = 0` (in-place rotation) | `EXPECT_EQ` |

## 4. Path Processor Tests

This section details the test cases for the `path_processor` library.

### 4.1. `Distance`
- **Objective:** To verify that the `distance` function correctly calculates the Euclidean distance between two points.

| Test Scenario | Inputs | Expected Outcome | Assertion(s) |
| :--- | :--- | :--- | :--- |
| Standard case | `p1={0,0}`, `p2={3,4}` | Distance is 5.0 | `EXPECT_DOUBLE_EQ` |
| Same point | `p1={0,0}`, `p2={0,0}` | Distance is 0.0 | `EXPECT_DOUBLE_EQ` |
| Negative coordinates | `p1={0,0}`, `p2={-3,-4}` | Distance is 5.0 | `EXPECT_DOUBLE_EQ` |

### 4.2. `CubicBezier`
- **Objective:** To verify the `cubic_bezier` function correctly calculates a point on a cubic Bezier curve for a given parameter `t`.

| Test Scenario | Inputs | Expected Outcome | Assertion(s) |
| :--- | :--- | :--- | :--- |
| Start of curve | `t=0.0` | Result is the start point `p0` | `EXPECT_DOUBLE_EQ` |
| End of curve | `t=1.0` | Result is the end point `p3` | `EXPECT_DOUBLE_EQ` |
| Midpoint of curve | `t=0.5` | Result is the calculated midpoint | `EXPECT_DOUBLE_EQ` |

### 4.3. `CalculateControlPoints`
- **Objective:** To ensure the `calculate_control_points` function correctly computes Bezier control points (`p1`, `p2`) for a given set of waypoints.

| Test Scenario | Inputs | Expected Outcome | Assertion(s) |
| :--- | :--- | :--- | :--- |
| Standard case | 3 waypoints | Correct `p1` and `p2` vectors are generated | `ASSERT_EQ`, `EXPECT_NEAR` |
| Edge case: 2 waypoints | 2 waypoints | Correct `p1` and `p2` vectors are generated | `ASSERT_EQ`, `EXPECT_NEAR` |
| Edge case: Empty | 0 waypoints | `p1` and `p2` vectors are empty | `EXPECT_TRUE` |
| Edge case: 1 waypoint | 1 waypoint | `p1` and `p2` vectors are empty | `EXPECT_TRUE` |

### 4.4. `GenerateSmoothPath`
- **Objective:** To validate that `generate_smooth_path` produces a plausible, dense path from a sparse set of waypoints.

| Test Scenario | Inputs | Expected Outcome | Assertion(s) |
| :--- | :--- | :--- | :--- |
| Straight line | Waypoints for a straight line | Path size is as expected; start/end points match | `EXPECT_NEAR`, `EXPECT_DOUBLE_EQ` |
| Edge case: Empty | 0 waypoints | The generated path is empty | `EXPECT_TRUE` |
| Edge case: 1 waypoint | 1 waypoint | The generated path is empty | `EXPECT_TRUE` |

### 4.5. `GenerateTimedPath`
- **Objective:** To verify that `generate_timed_path` correctly assigns timestamps, yaws, and velocities to a given path.

| Test Scenario | Inputs | Expected Outcome | Assertion(s) |
| :--- | :--- | :--- | :--- |
| Straight path | Path with equidistant points, `desired_velocity=0.5` | Timestamps increase linearly; yaw is constant; computed velocity matches desired velocity | `ASSERT_EQ`, `EXPECT_DOUBLE_EQ`, `EXPECT_NEAR` |
| Edge case: Empty | Empty path | The generated timed path is empty | `EXPECT_TRUE` |
| Edge case: 1 point | Path with one point | A single timed pose is generated with 0 time offset and 0 yaw | `ASSERT_EQ`, `EXPECT_DOUBLE_EQ` |

## 5. Coverage
The test suites are designed to achieve high line coverage for `path_tracker_node.cpp` and `path_processor.cpp`, ensuring that all critical logic paths are exercised.
