# Extending smooth_path_tracker to a Real Robot

This document outlines practical steps, considerations, and recommended changes to deploy `smooth_path_tracker` from simulation or development rigs to a physical robot (e.g., TurtleBot3 or a custom differential-drive platform).

## Summary of changes required

1. Hardware interface and drivers
2. Sensor integration and state estimation
3. Safety and watchdog systems
4. Calibration and tuning
5. Real-time performance and hardware constraints
6. Testing plan and validation

## 1. Hardware interface and drivers

- Ensure a reliable low-latency command interface to the robot base. For TurtleBot3, interact with the standard `ros2_control` / `robot_state_publisher` or the stock driver that accepts velocity commands on `/cmd_vel` or `cmd_vel_unstamped`.
- If your platform uses a different command interface (e.g., serial over USB, custom CAN bus), implement a small bridge node that converts `geometry_msgs/TwistStamped` into the robot's native format.
- Respect command watchdogs: many low-level motor controllers require continuous commands and will stop if they don't receive them. Implement a command publisher timer and a safe stop behavior if the tracker node fails.

## 2. Sensor integration and state estimation

- Odometry: preferred source is fused odometry from wheel encoders + IMU using an EKF (e.g., `robot_localization`). Wheel encoders alone may drift; fuse IMU and visual odometry where possible.
- IMU: provides orientation and angular rates for better yaw estimation and to detect slips.
- Localization: when operating in a mapped environment, integrate with AMCL or LIO/SLAM systems for global pose.
- TF: Ensure TF frames are published and that the `base_footprint` -> `odom` -> `map` chain is available. The tracker should rely on the `base_footprint` pose in the `odom` frame for smooth, continuous tracking, while the path itself is typically defined in the `map` frame. The `path_tracker_node` does not handle the transformation itself; it assumes the odometry and path are in a consistent frame.

## 3. Safety and watchdogs

- Emergency stop (E-Stop): subscribe to or integrate with an E-Stop topic/mechanism. On E-Stop, immediately publish zero velocities and block further commands until cleared.
- Obstacle detection and collision avoidance: integrate with a local planner or obstacle monitor (e.g., `costmap_2d` + `dwa_local_planner` or a custom safety node) that can preempt or modify velocity commands if obstacles are detected.
- Soft limits: cap `max_linear_velocity`, `max_angular_velocity`, and acceleration to values safe for the platform.
- Runtime liveness: monitor node health and implement a watchdog that triggers a safe stop if the node crashes or becomes unresponsive.

## 4. Calibration and tuning

- Kinematic parameters: wheel base, wheel radius, and any transform offsets must be accurate.
- PID/tuning for low-level controllers (if present) should be tuned to accept velocity commands and produce accurate motion.
- **Tune Controller Parameters**: The pure pursuit controller's behavior is highly dependent on its parameters.
  - `lookahead_distance`: This is the most critical parameter. A small value (e.g., 0.2-0.4m) will make the robot follow the path very closely but can lead to oscillations. A larger value (e.g., 0.5-1.0m) provides smoother control but may cause the robot to cut corners. A good starting point is 1.5 to 2 times the robot's length.
  - `max_linear_velocity`: Start with a safe, low speed (e.g., 0.1-0.2 m/s) and increase it as you gain confidence in the robot's tracking performance.
  - `yaw_error_threshold`: This determines when the robot performs an in-place rotation. A smaller value will make the robot prioritize being aligned with the path, which is safer in tight spaces. A larger value allows for more fluid, sweeping turns.
  - `goal_tolerance`: This should be set based on the required precision for the task. A value of 5-10 cm is typical.

## 5. Real-time performance and hardware constraints

- Node scheduling: use a real-time enabled kernel if precise control loops are required, or ensure the tracker runs with sufficient QoS in ROS2 and an adequate CPU budget.
- Use efficient math libraries and avoid heavy allocations in the control loop. Pre-allocate buffers if necessary.
- Consider running path smoothing offline or less frequently; run tracking at a fixed high-rate loop (e.g., 20–50 Hz) to ensure responsiveness.

## 6. Testing plan and validation

- Unit tests: ensure algorithms behave as expected (already present in `test/`).
- Simulation: test in Gazebo or Ignition with realistic robot models and sensors. Use replayed sensor logs (rosbag) from the physical robot to validate.
- On-robot staged testing:
  1. Tethered motor tests with low-speed commands and a safety kill switch.
  2. Slow open-loop tests to confirm odometry direction and sign conventions.
  3. Closed-loop tests in a controlled environment (soft barriers, low speed).
  4. Field tests with full autonomy after increasing confidence.

## 7. Path Generation on a Real Robot

The `path_smoother_node` is currently configured to generate a fixed path from a hardcoded list of waypoints defined in `params.yaml`. For a real robot, you will likely want to generate paths dynamically. Here are some common approaches:

- **Global Planner Integration**: In a full navigation stack, a global planner (like the one in Nav2) would generate a path to a goal. You would replace the `path_smoother_node` with a subscriber to the global planner's path topic. You could still use the `PathProcessor` class to smooth this path before sending it to the `path_tracker_node`.
- **GUI-Based Waypoint Selection**: You can use RViz's "Publish Point" feature to send goal poses. A simple Python script could collect these poses and then use the `PathProcessor` to generate and publish a smooth path connecting them.
- **Pre-recorded Paths**: For repeatable tasks, you can drive the robot manually and record its trajectory using `ros2 bag`. A script can then process this bag file to extract the path, smooth it, and publish it.

## Additional integrations

- Bringup launch file: create a production-ready bringup launch that starts the tracker with proper parameter files, remappings, and safety nodes.
- Data logging: enable rosbag logging of `/odom`, `/cmd_vel`, `/scan`, TF, and other relevant topics for offline analysis.
- Monitoring dashboards: use rqt, RViz, or custom dashboards to display current state, lookahead point, errors, and commands.

## Example production parameter changes

- Reduce `max_linear_velocity` from default 0.22 to a safe initial value like 0.1 m/s for first tests.
- Set `yaw_error_threshold` to a conservative 0.5 rad to prefer in-place rotations during early testing.
 

## Summary

Deploying `smooth_path_tracker` to a real robot requires attention to hardware interfaces, accurate state estimation, safety systems, parameter tuning, and a rigorous testing plan. The tracker itself is primarily a geometric controller and can be robust on real hardware when supplied with reliable pose estimates and conservative parameters.
