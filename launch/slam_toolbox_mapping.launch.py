import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
    default_params = os.path.join(repo_root, 'config', 'slam_toolbox_mapping.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='slam_toolbox mapping parameter file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Set true only when /clock is published by Gazebo or a bridge',
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
            remappings=[
                ('/scan', '/lidar'),
                ('scan', '/lidar'),
            ],
        ),
    ])
