#!/usr/bin/env python3
import math
from status_model import peripheral_status
from status_store import read, update

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
        super().__init__(getattr(args, "node_name", "health_monitor"))

        self.args = args
        self.start_time = time.monotonic()

        self.last_fix_time = None
        self.last_heading_time = None
        self.last_lidar_time = None
        self.last_ultrasonic_time = None
        self.last_can_time = None
        self.last_can_valid = False
        self.invalid_data = {}

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

        self.get_logger().info("Health monitor started")
        self.get_logger().info(f"check_fix={args.check_fix}, topic={args.fix_topic}")
        self.get_logger().info(f"check_heading={args.check_heading}, topic={args.heading_topic}")
        self.get_logger().info(f"check_lidar={args.check_lidar}, topic={args.lidar_topic}")
        self.get_logger().info(f"check_ultrasonic={args.check_ultrasonic}, topic={args.ultrasonic_topic}")
        self.get_logger().info(f"check_can={args.check_can}, topic={args.can_feedback_topic}")
        self.get_logger().info(
            f"startup_timeout={args.startup_timeout}s, runtime_timeout={args.runtime_timeout}s"
        )

    def fix_cb(self, msg: NavSatFix):
        valid = (msg.status.status >= 0 and math.isfinite(msg.latitude)
                 and math.isfinite(msg.longitude) and -90 <= msg.latitude <= 90
                 and -180 <= msg.longitude <= 180)
        self.invalid_data['fix'] = '' if valid else '无定位或经纬度无效'
        if valid:
            self.last_fix_time = time.monotonic()

    def heading_cb(self, msg: Float32):
        valid = math.isfinite(msg.data)
        self.invalid_data['heading'] = '' if valid else '航向不是有限数值'
        if valid:
            self.last_heading_time = time.monotonic()

    def lidar_cb(self, msg: PointCloud2):
        valid = msg.width * msg.height > 0 and bool(msg.data)
        self.invalid_data['lidar'] = '' if valid else '点云为空'
        if valid:
            self.last_lidar_time = time.monotonic()

    def ultrasonic_cb(self, msg: Float32MultiArray):
        valid = len(msg.data) >= 4 and all(math.isfinite(v) for v in msg.data[:4])
        self.invalid_data['ultrasonic'] = '' if valid else '不足四路或包含非有限数值'
        if valid:
            self.last_ultrasonic_time = time.monotonic()

    def can_feedback_cb(self, msg: CanFeedback):
        self.last_can_time = time.monotonic()
        self.last_can_valid = (msg.rpm_left_valid or msg.rpm_right_valid
                               or msg.throttle_left_valid or msg.throttle_right_valid)
        self.invalid_data['can'] = '' if self.last_can_valid else '电机 RPM 和油门反馈均无效'

    def peripheral_status(self, timeout, starting=False):
        checks = {}
        if self.args.check_fix or self.args.check_heading:
            checks['rtk'] = []
            if self.args.check_fix:
                checks['rtk'].append(('RTK_FIX', 'RTK 定位', self.last_fix_time, self.invalid_data.get('fix')))
            if self.args.check_heading:
                checks['rtk'].append(('RTK_HEADING', 'RTK 航向', self.last_heading_time, self.invalid_data.get('heading')))
        for key, enabled, label, last in (
            ('lidar', self.args.check_lidar, '激光雷达点云', self.last_lidar_time),
            ('ultrasonic', self.args.check_ultrasonic, '超声波', self.last_ultrasonic_time),
            ('can', self.args.check_can, 'CAN/VCU', self.last_can_time),
        ):
            if enabled:
                checks[key] = [(key.upper(), label, last, self.invalid_data.get(key))]
        return peripheral_status(checks, time.monotonic(), timeout, starting)

    def missing_reasons(self, timeout):
        return [error['message'] for item in self.peripheral_status(timeout).values()
                for error in item['errors']]

    def shutdown_launch(self, title, reasons, exit_code):
        update("runtime_fault" if exit_code == 3 or read().get("ever_ready") else "startup_failed", "；".join(reasons))
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
        if exit_code:
            update("startup_failed", "；".join(reasons))
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
        elapsed = time.monotonic() - self.start_time

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

        self.get_logger().info("Startup health check passed, switching to runtime monitor")

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

    parser.add_argument("--startup-timeout", type=float, default=8.0)
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
