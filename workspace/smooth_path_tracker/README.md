# Smooth Path Tracker

## Overview

This ROS2 package provides functionalities for path smoothing and tracking for a mobile robot, specifically tailored for the Turtlebot3. The package includes nodes for path processing, smoothing, and tracking.

## Features

- **Path Processing**: Processes raw paths to prepare them for smoothing and tracking.
- **Path Smoothing**: Implements algorithms to smooth the given path to make it feasible for a robot to follow.
- **Path Tracking**: A controller to make the robot follow the smoothed path accurately.
- **Visualization**: Publishes markers to visualize the path and other relevant information in RViz.

## Dependencies

This package depends on the following ROS2 packages:
- `rclcpp`
- `nav_msgs`
- `geometry_msgs`
- `tf2`
- `tf2_ros`
- `visualization_msgs`

## Installation

1. Clone the repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```
2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select smooth_path_tracker
   ```
3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

To launch the path follower, run the following command:

```bash
ros2 launch smooth_path_tracker path_follower.launch.py
```

## How to Test

To run the tests for this package, use the following command from the root of the workspace:

```bash
colcon test --packages-select smooth_path_tracker
```

To see the test results, you can run:

```bash
colcon test-result --verbose
```

## License

This project is licensed under the Apache-2.0 License. See the [LICENSE](LICENSE) file for details.
