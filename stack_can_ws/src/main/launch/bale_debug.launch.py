#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='/home/xinye30/stack_can_ws/src/main/config/params.yaml',
            description='Path to params.yaml'
        ),

        # -------------------------------------------------
        # 1) 雷达驱动
        # -------------------------------------------------
        Node(
            package='robosense_driver',
            executable='rsview',
            name='rsview',
            output='screen',
            parameters=[params_file],
        ),

        # -------------------------------------------------
        # 2) 草捆检测
        # 需要发布 /bale_target
        # -------------------------------------------------
        Node(
            package='robosense_driver',
            executable='bale_detector',
            name='bale_detector',
            output='screen',
            parameters=[params_file],
        ),

        # -------------------------------------------------
        # 3) CAN 执行器
        # -------------------------------------------------
        Node(
            package='main',
            executable='stack_can_executor',
            name='stack_can_executor',
            output='screen',
            parameters=[params_file],
        ),

        # -------------------------------------------------
        # 4) 草捆对准控制器（新版本）
        # -------------------------------------------------
        Node(
            package='main',
            executable='bale_align_controller',
            name='bale_align_controller',
            output='screen',
            parameters=[params_file, {
                'angle_lpf_alpha': 0.25,
                'align_enter_deg': 2.0,
                'align_exit_deg': 6.0,

                'turn_cmd_angle_deg': 10.0,
                'turn_frame_period_s': 0.10,
                'turn_pulse_frames': 5,
                'turn_pause_frames': 1,
                'turn_cycle_yaw_deg': 2.6,

                'approach_dist_m': 2.0,
                'approach_speed_kmh': 4.0,

                'too_close_dist_m': 1.0,
                'retreat_speed_kmh': 4.0,

                'target_timeout_s': 0.50,
                'pick_action_hold_s': 30.0,
            }],
        ),

        # -------------------------------------------------
        # 5) 调试时强制状态
        # 不走 traj，但让 bale_controller 认为自己处在“任务等待点”
        # -------------------------------------------------

        # /drive_cmd = 1 (RUNNING)
        ExecuteProcess(
            cmd=[
                'bash', '-lc',
                'source /opt/ros/humble/setup.bash && '
                'source /home/xinye30/stack_can_ws/install/setup.bash && '
                'ros2 topic pub -r 5 /drive_cmd std_msgs/msg/UInt8 "{data: 1}"'
            ],
            output='screen'
        ),

        # /at_task_waiting = true
        ExecuteProcess(
            cmd=[
                'bash', '-lc',
                'source /opt/ros/humble/setup.bash && '
                'source /home/xinye30/stack_can_ws/install/setup.bash && '
                'ros2 topic pub -r 5 /at_task_waiting std_msgs/msg/Bool "{data: true}"'
            ],
            output='screen'
        ),

        # /teleop_active = false
        # 否则 executor 可能优先吃 teleop
        ExecuteProcess(
            cmd=[
                'bash', '-lc',
                'source /opt/ros/humble/setup.bash && '
                'source /home/xinye30/stack_can_ws/install/setup.bash && '
                'ros2 topic pub -r 2 /teleop_active std_msgs/msg/Bool "{data: false}"'
            ],
            output='screen'
        ),

        # /bale_active 可由 controller 自己发，不需要这里额外发布
    ])
