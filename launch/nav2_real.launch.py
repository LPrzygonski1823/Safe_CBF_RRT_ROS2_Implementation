"""Minimal Nav2 for the Safe-CBF-RRT* planner on a physical ROSbot XL (Jazzy).

planner_node bypasses bt_navigator and planner_server and talks to controller_server
(FollowPath) directly, so only these are started:
  slam_toolbox -> /map and map->odom
  controller_server (+ local costmap) -> cmd_vel_nav
  velocity_smoother -> /autonomous/cmd_vel (TwistStamped, priority 10 in the snap's twist_mux)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')

    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
            'autostart': 'true',
        }.items())

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')])

    smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', '/autonomous/cmd_vel')])

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'bond_timeout': 8.0,
                     'node_names': ['controller_server', 'velocity_smoother']}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('slam_params_file'),
        slam,
        controller,
        smoother,
        lifecycle,
    ])
