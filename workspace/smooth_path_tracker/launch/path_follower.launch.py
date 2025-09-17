from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    params_filename_arg = DeclareLaunchArgument(
        'params_filename',
        default_value='params.yaml',
        description='Name of the parameter file'
    )
    rviz_filename_arg = DeclareLaunchArgument(
        'rviz_filename',
        default_value='path_follower.rviz',
        description='Name of the rviz config file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_filename = LaunchConfiguration('params_filename')
    rviz_filename = LaunchConfiguration('rviz_filename')

    smooth_path_tracker_pkg_share = FindPackageShare(package='smooth_path_tracker')
    params_file = PathJoinSubstitution([smooth_path_tracker_pkg_share, 'config', params_filename])
    rviz_file = PathJoinSubstitution([smooth_path_tracker_pkg_share, 'config', rviz_filename])

    path_smoother = Node(
        package='smooth_path_tracker',
        executable='path_smoother_node',
        name='path_smoother_node',
        parameters=[params_file, { 'use_sim_time': use_sim_time }],
        output='screen'
    )
    
    path_tracker = Node(
        package='smooth_path_tracker',
        executable='path_tracker_node',
        name='path_tracker_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    nodes = [path_smoother, path_tracker, rviz2]
    args = [use_sim_time_arg, params_filename_arg, rviz_filename_arg]

    return LaunchDescription(args + nodes)