# ROS2 Smooth Path Tracker Workspace

This workspace contains the `smooth_path_tracker`, a ROS2 package for path smoothing and tracking for mobile robots. For more details on the package itself, see the package's [README.md](workspace/smooth_path_tracker/README.md).

## Docker Setup

To set up the development environment, follow these steps from the `scripts` directory:

1.  **Build the Docker Image:**
    ```bash
    ./build_img.sh
    ```

2.  **Start the Development Container:**
    ```bash
    ./devel.sh
    ```

3.  **Enter the Container:**
    For each new terminal, run the following command to get a shell inside the container:
    ```bash
    ./enter_bash.sh
    ```

The container already has the workspace built and sourced. You can start working on the code immediately.

## Getting Started

### How to Build

To build the packages in this workspace, run the following commands from the root of the workspace:

```bash
colcon build --symlink-install
```

### How to Run

To launch the path follower, first source the workspace:

1.  Source the workspace:
    ```bash
    source install/setup.bash
    ```

2.  Launch Gazebo with Turtlebot3:
    ```bash
    export TURTLEBOT3_MODEL=waffle_pi
    ros2 launch turtlebot3_gazebo empty_world.launch.py
    ```

4. Setup Plotjuggler (optional):
    ```bash
    ros2 run plotjuggler_ros plotjuggler
    ```
    Plotjuggler config is in [tracking_performance.xml](workspace/smooth_path_tracker/config/tracking_performance.xml)

3.  Then, run the launch file:
    ```bash
    ros2 launch smooth_path_tracker path_follower.launch.py
    ```

## Documentation

For more details on the design and how to extend the functionality to a real robot, please see the following documents:

- [Design Document](workspace/smooth_path_tracker/docs/DESIGN.md)
- [Extending to a Real Robot](workspace/smooth_path_tracker/docs/EXTENDING_TO_REAL_ROBOT.md)

## Demonstration
A demonstration of the `smooth_path_tracker` in action can be found in the following video:

<video src="workspace/smooth_path_tracker/docs/End-to-EndDemo.webm" width="640" height="480" controls autoplay loop>
  Your browser does not support the video tag.
</video>

## Development Note
This project was developed with the assistance of Co-pilot with gemini 2.5 pro agent mode.
