import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('cbf_rrt_planner'), 'config', 'cbf_rrt_planner.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        # the Gazebo stack publishes /clock and Nav2 runs with use_sim_time:=True - on the
        # physical robot there is no /clock at all
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='true for the Gazebo stack, false on the physical ROSbot XL'),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='planner parameters (kappa, safety_weight_c, nominal_velocity, ...)'),

        Node(
            package='cbf_rrt_planner',
            executable='planner_node',
            name='planner_node',
            output='screen',
            emulate_tty=True,
            # listed last so that the launch argument wins over the value in the yaml file
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
