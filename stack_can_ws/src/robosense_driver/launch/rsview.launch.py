from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robosense_driver',
            executable='rsview',
            name='robosense_driver',
            output='screen',
            arguments=[
                '-host', '192.168.1.102',
                '-msop', '6699',
                '-difop', '7788',
            ]
        )
    ])
