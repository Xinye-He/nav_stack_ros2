#!/usr/bin/env python3
import argparse
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import NavSatFix, PointCloud2
from std_msgs.msg import Float32, Float32MultiArray
from stack_msgs.msg import CanFeedback


class StartupHealthCheck(Node):
    def __init__(self, args):
        super().__init__("startup_health_check")

        self.args = args
        self.start_time = time.time()

        self.last_fix_time = None
        self.last_heading_time = None
        self.last_lidar_time = None
        self.last_ultrasonic_time = None
        self.last_can_time = None
        self.last_can_valid = False

        qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        if args.check_fix:
            self.create_subscription(
                NavSatFix,
                args.fix_topic,
                self.fix_cb,
                qos_sensor
            )

        if args.check_heading:
            self.create_subscription(
                Float32,
                args.heading_topic,
                self.heading_cb,
                qos_sensor
            )

        if args.check_lidar:
            self.create_subscription(
                PointCloud2,
                args.lidar_topic,
                self.lidar_cb,
                qos_sensor
            )

        if args.check_ultrasonic:
            self.create_subscription(
                Float32MultiArray,
                args.ultrasonic_topic,
                self.ultrasonic_cb,
                qos_sensor
            )

        if args.check_can:
            self.create_subscription(
                CanFeedback,
                args.can_feedback_topic,
                self.can_feedback_cb,
                10
            )

        self.timer = self.create_timer(0.5, self.check_once)

        self.get_logger().info("Startup health check started")
        self.get_logger().info(f"check_fix={args.check_fix}, topic={args.fix_topic}")
        self.get_logger().info(f"check_heading={args.check_heading}, topic={args.heading_topic}")
        self.get_logger().info(f"check_lidar={args.check_lidar}, topic={args.lidar_topic}")
        self.get_logger().info(f"check_ultrasonic={args.check_ultrasonic}, topic={args.ultrasonic_topic}")
        self.get_logger().info(f"check_can={args.check_can}, topic={args.can_feedback_topic}")
        self.get_logger().info(
            f"startup_timeout={args.startup_timeout}s, runtime_timeout={args.runtime_timeout}s"
        )

    def fix_cb(self, msg: NavSatFix):
        # NavSatStatus.STATUS_NO_FIX == -1
        if msg.status.status >= 0:
            self.last_fix_time = time.time()

    def heading_cb(self, msg: Float32):
        self.last_heading_time = time.time()

    def lidar_cb(self, msg: PointCloud2):
        self.last_lidar_time = time.time()

    def ultrasonic_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 4 and any(v > 0.0 for v in msg.data[:4]):
            self.last_ultrasonic_time = time.time()

    def can_feedback_cb(self, msg: CanFeedback):
        self.last_can_time = time.time()
        self.last_can_valid = (
            msg.rpm_left_valid or
            msg.rpm_right_valid or
            msg.throttle_left_valid or
            msg.throttle_right_valid
        )

    def missing_reasons(self, timeout):
        now = time.time()
        reasons = []

        if self.args.check_fix:
            if self.last_fix_time is None or now - self.last_fix_time > timeout:
                reasons.append(
                    f"RTK/GPS 无有效 {self.args.fix_topic} 数据。"
                    f"请检查 RTK 是否上电、天线、串口、NMEA GGA/RMC 输出。"
                )

        if self.args.check_heading:
            if self.last_heading_time is None or now - self.last_heading_time > timeout:
                reasons.append(
                    f"RTK 航向角无有效 {self.args.heading_topic} 数据。"
                    f"请检查 THS/HDT 输出、双天线定向、RTK 配置。"
                )

        if self.args.check_lidar:
            if self.last_lidar_time is None or now - self.last_lidar_time > timeout:
                reasons.append(
                    f"激光雷达无有效 {self.args.lidar_topic} 点云。"
                    f"请检查雷达是否上电、网线、主机 IP、msop/difop 端口、雷达 IP。"
                )

        if self.args.check_ultrasonic:
            if self.last_ultrasonic_time is None or now - self.last_ultrasonic_time > timeout:
                reasons.append(
                    f"超声波无有效 {self.args.ultrasonic_topic} 数据。"
                    f"请检查 ultrasonic 节点是否启动、串口设备是否存在、传感器是否上电、波特率是否正确。"
                )

        if self.args.check_can:
            if self.last_can_time is None or now - self.last_can_time > timeout:
                reasons.append(
                    f"CAN 状态无有效 {self.args.can_feedback_topic} 数据。"
                    f"请检查 can_feedback_node 是否启动、can0 是否 up、VCU 是否上电、CAN 接线和终端电阻。"
                )
            elif not self.last_can_valid:
                reasons.append(
                    f"CAN 状态话题 {self.args.can_feedback_topic} 有发布，但左右电机 RPM/油门反馈均无效。"
                    f"请检查 VCU 是否发送反馈帧，以及 CAN ID 是否与参数配置匹配。"
                )

        return reasons

    def shutdown_launch(self, title, reasons, exit_code):
        self.get_logger().error("")
        self.get_logger().error(title)
        for reason in reasons:
            self.get_logger().error(reason)
        self.get_logger().error("=" * 60)

        # 关键：直接给 ros2 launch 父进程发送 SIGINT
        parent_pid = os.getppid()
        self.get_logger().error(
            f"Sending SIGINT to parent ros2 launch process, ppid={parent_pid}"
        )

        try:
            os.kill(parent_pid, signal.SIGTERM)
        except Exception as e:
            self.get_logger().error(f"Failed to send SIGINT to parent process: {e}")

        # 给 launch 一点时间开始关闭其他节点
        time.sleep(1.0)

        # 自己强制退出，确保不会继续运行
        os._exit(exit_code)

    def exit_with_status(self, title, reasons, exit_code):
        if reasons:
            self.get_logger().error("")
            self.get_logger().error(title)
            for reason in reasons:
                self.get_logger().error(reason)
            self.get_logger().error("=" * 60)
        else:
            self.get_logger().info(title)

        time.sleep(0.2)
        os._exit(exit_code)

    def check_once(self):
        elapsed = time.time() - self.start_time

        if elapsed < self.args.startup_timeout:
            return

        reasons = self.missing_reasons(self.args.startup_timeout)

        if reasons:
            if self.args.once:
                self.exit_with_status(
                    "========== DEMO 启动检查失败，demo.launch.py 不会启动 ==========",
                    reasons,
                    2
                )

            self.shutdown_launch(
                "========== DEMO 启动失败，主动终止 ==========",
                reasons,
                2
            )

        self.get_logger().info("Startup health check passed")

        if self.args.once:
            self.exit_with_status(
                "========== DEMO 启动检查通过 ==========",
                [],
                0
            )

        self.timer.cancel()
        self.timer = self.create_timer(1.0, self.runtime_check)

    def runtime_check(self):
        reasons = self.missing_reasons(self.args.runtime_timeout)

        if reasons:
            self.shutdown_launch(
                "========== DEMO 运行中健康检查失败，主动终止 ==========",
                reasons,
                3
            )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--check-fix", action="store_true")
    parser.add_argument("--check-heading", action="store_true")
    parser.add_argument("--check-lidar", action="store_true")
    parser.add_argument("--check-ultrasonic", action="store_true")
    parser.add_argument("--check-can", action="store_true")

    parser.add_argument("--fix-topic", default="/fix")
    parser.add_argument("--heading-topic", default="/heading_deg")
    parser.add_argument("--lidar-topic", default="/rslidar_points")
    parser.add_argument("--ultrasonic-topic", default="/ultrasonic_distances")
    parser.add_argument("--can-feedback-topic", default="/stack_can/feedback")

    parser.add_argument("--startup-timeout", type=float, default=3.0)
    parser.add_argument("--runtime-timeout", type=float, default=2.0)
    parser.add_argument(
        "--once",
        action="store_true",
        help="Run only the startup check and exit without signalling the parent process."
    )

    args = parser.parse_args()

    rclpy.init()
    node = StartupHealthCheck(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
