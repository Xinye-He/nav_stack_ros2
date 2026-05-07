from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('main')

    default_yaml = os.path.join(pkg_share, 'config', 'params.yaml')
    default_yaml_env = EnvironmentVariable(name='GPS_PID_PARAMS_FILE', default_value=default_yaml)

    params_file = LaunchConfiguration('params_file')
    enable_bale_pipeline = LaunchConfiguration('enable_bale_pipeline')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_yaml_env,
            description='YAML parameters file'
        ),
        DeclareLaunchArgument(
            'enable_bale_pipeline',
            default_value='true',
            description='Launch bale detector + bale align controller'
        ),

        # ---------------- 基础定位 / 循迹链 ----------------
        Node(
            package='main',
            executable='can_feedback_node',
            name='can_feedback_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='main',
            executable='dr_odometry_node',
            name='dr_odometry',
            parameters=[{
                'speed_topic': '/ground_speed_mps',
                'heading_topic': '/vehicle_heading_deg',
                'yaw_rate_topic': '/wheel_yaw_rate_rad_s',
                'odom_topic': '/dr/odom',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'use_heading': True,
                'use_yaw_rate': True,
                'yaw_offset_deg': 0.0,
                'publish_tf': True,
                'rate_hz': 50.0
            }],
            output='screen'
        ),

        Node(
            package='main',
            executable='traj_waypoint_follower',
            name='traj_waypoint_follower',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='main',
            executable='stack_can_executor',
            name='stack_can_executor',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='main',
            executable='geofence_monitor',
            name='geofence_monitor',
            output='screen',
            parameters=[params_file],
        ),

        # ---------------- 草捆链（正式流程） ----------------
        Node(
            package='robosense_driver',
            executable='bale_detector',
            name='bale_detector',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_bale_pipeline),
        ),

        Node(
            package='main',
            executable='bale_align_controller',
            name='bale_align_controller',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_bale_pipeline),
        ),
    ])
