from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='smooth_path_tracker',
            executable='path_smoother_node',
            name='path_smoother_node',
            parameters=[{ 'use_sim_time': True }],
            output='screen'
        ),
        Node(
            package='smooth_path_tracker',
            executable='path_tracker_node',
            name='path_tracker_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'lookahead_distance': 0.3,
                'linear_velocity': 0.5,
                'goal_tolerance': 0.1
            }]
        ),
    ])