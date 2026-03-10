from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('main')

    # 参数文件（优先级：命令行 > 环境变量 > 默认路径）
    default_yaml = os.path.join(pkg_share, 'config', 'params.yaml')
    default_yaml_env = EnvironmentVariable(name='GPS_PID_PARAMS_FILE', default_value=default_yaml)
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_yaml_env,
            description='YAML parameters file for traj_waypoint_follower'
        ),

        # IMU（发布 imu/data_raw；你已修改：应发布 base_link->imu_link 静态TF；不要发布 odom->base_link）
        Node(
            package='imu_driver',
            executable='imu_driver',
            name='imu',
            remappings=[('/imu/data_raw', '/imu/data')],
            parameters=[{'port': '/dev/imu_usb'}, {'baud': 9600}],
            output='screen'
        ),

        # GPS/NMEA bridge：输出 /fix(主天线), /heading_deg(主->副基线), /ground_speed_mps
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

        # RTK 双天线：把主天线点修正到车身中心 + 输出车头航向
        # 注意：方案B中不让它发TF（避免与 follower/dr_odometry 冲突）
        Node(
            package='main',
            executable='rtk_center_from_nmea',  # 确保 setup.py 里 console_scripts 名字一致
            name='rtk_center_from_nmea',
            parameters=[{
                'fix_topic': '/fix',
                'heading_topic': '/heading_deg',
                'out_fix_center_topic': '/fix_center',
                'out_vehicle_heading_topic': '/vehicle_heading_deg',

                # 车体参数：主天线（前左角） -> 车身中心（base_link）
                'main_to_center_x': -1.35,
                'main_to_center_y': -0.75,
                'main_to_center_z': 0.0,

                # THS解释：0=北，顺时针；heading 是 主->副
                'heading_from_north_cw': True,
                'heading_offset_deg': 0.0,

                # 主在左、副在右：基线指向车体右侧(-Y)，车头(+X)=基线+90°
                'baseline_to_vehicle_yaw_offset_deg': 90.0,

                # 方案B：不发布 map->base_link TF
                'publish_tf': False,
                'set_origin_on_first_center_fix': False,
            }],
            output='screen'
        ),

        # DR里程计：发布 odom->base_link（连续），符合 REP-105
        Node(
            package='main',
            executable='dr_odometry_node',  # 确保 setup.py 里 console_scripts 名字一致
            name='dr_odometry',
            parameters=[{
                'speed_topic': '/ground_speed_mps',
                'heading_topic': '/vehicle_heading_deg',
                'odom_topic': '/dr/odom',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
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
            parameters=[LaunchConfiguration('params_file')],
        ),


        Node(
            package='main',
            executable='stack_can_executor',
            name='stack_can_executor',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),

        Node(
            package='main',
            executable='websocket_teleop_key',
            name='websocket_teleop_key',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
