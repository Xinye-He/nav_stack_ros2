from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robosense_driver')

        Node(
            package='robosense_driver',
            executable='rsview',
            name='rsview',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),

        Node(
            package='robosense_driver',
            executable='bale_detector',
            name='bale_detector',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
