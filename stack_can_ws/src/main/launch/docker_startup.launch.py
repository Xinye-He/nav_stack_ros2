from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
    enable_lidar_driver = LaunchConfiguration('enable_lidar_driver')
    enable_ws_server = LaunchConfiguration('enable_ws_server')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_yaml_env,
            description='YAML parameters file'
        ),
        DeclareLaunchArgument(
            'enable_lidar_driver',
            default_value='true',
            description='Launch robosense lidar driver'
        ),
        DeclareLaunchArgument(
            'enable_ws_server',
            default_value='true',
            description='Launch GPS / heading / CSV websocket server'
        ),

        Node(
            package='imu_driver',
            executable='imu_driver',
            name='imu',
            remappings=[('/imu/data_raw', '/imu/data')],
            parameters=[{'port': '/dev/imu_usb'}, {'baud': 9600}],
            output='screen'
        ),

        Node(
            package='nmea_bridge',
            executable='nmea_bridge_node',
            name='nmea_bridge',
            parameters=[{
                'port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
                'baudrate': 115200,
                'frame_id': 'gps_main',
                'check_crc': True,
                'min_satellites': 0
            }],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'python3',
                '/root/stack_can_ws/script/server_all_ws.py'
            ],
            name='server_all_ws',
            output='screen',
            condition=IfCondition(enable_ws_server),
        ),

        Node(
            package='main',
            executable='rtk_center_from_nmea',
            name='rtk_center_from_nmea',
            parameters=[{
                'fix_topic': '/fix',
                'heading_topic': '/heading_deg',
                'out_fix_center_topic': '/fix_center',
                'out_vehicle_heading_topic': '/vehicle_heading_deg',
                'main_to_center_x': -1.35,
                'main_to_center_y': -0.75,
                'main_to_center_z': 0.0,
                'heading_from_north_cw': True,
                'heading_offset_deg': 0.0,
                'baseline_to_vehicle_yaw_offset_deg': 90.0,
                'publish_tf': False,
                'set_origin_on_first_center_fix': False,
            }],
            output='screen'
        ),

        Node(
            package='robosense_driver',
            executable='rsview',
            name='rsview',
            output='screen',
            parameters=[params_file],
            arguments=[
                '-host', '192.168.1.102',
                '-msop', '6699',
                '-difop', '7788',
            ],
            condition=IfCondition(enable_lidar_driver),
        ),
    ])
