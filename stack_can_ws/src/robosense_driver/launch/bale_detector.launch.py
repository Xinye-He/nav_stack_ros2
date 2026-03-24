from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robosense_driver')

    default_sensor = os.path.join(pkg_share, 'config', 'sensor.yaml')
    default_bale = os.path.join(pkg_share, 'config', 'bale_60x100.yaml')

    sensor_file = LaunchConfiguration('sensor_file')
    bale_file = LaunchConfiguration('bale_file')
    host = LaunchConfiguration('host')
    msop = LaunchConfiguration('msop')
    difop = LaunchConfiguration('difop')

    return LaunchDescription([
        DeclareLaunchArgument('sensor_file', default_value=default_sensor),
        DeclareLaunchArgument('bale_file', default_value=default_bale),
        DeclareLaunchArgument('host', default_value='192.168.1.102'),
        DeclareLaunchArgument('msop', default_value='6699'),
        DeclareLaunchArgument('difop', default_value='7788'),

        Node(
            package='robosense_driver',
            executable='rsview',
            name='rsview',
            output='screen',
            arguments=[
                '-host', host,
                '-msop', msop,
                '-difop', difop,
            ],
            parameters=[sensor_file],
        ),

        Node(
            package='robosense_driver',
            executable='bale_detector',
            name='bale_detector',
            output='screen',
            parameters=[sensor_file, bale_file],
        ),
    ])
