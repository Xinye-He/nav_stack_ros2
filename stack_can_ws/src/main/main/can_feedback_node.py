#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32
from stack_msgs.msg import CanFeedback

try:
    import can
    HAVE_CAN = True
except Exception:
    HAVE_CAN = False


def parse_can_id(value, default: int) -> int:
    try:
        if isinstance(value, (int, float)):
            return int(value)
        return int(str(value), 0)
    except Exception:
        return int(default)


class CanFeedbackNode(Node):
    def __init__(self):
        super().__init__('can_feedback_node')

        self.declare_parameter('enable_can', True)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_extended', True)

        self.declare_parameter('left_rpm_can_id', 0x0CF11E05)
        self.declare_parameter('right_rpm_can_id', 0x0CF11E06)
        self.declare_parameter('left_throttle_can_id', 0x0CF1051E)
        self.declare_parameter('right_throttle_can_id', 0x0CF1061E)

        self.declare_parameter('wheel_radius_m', 0.315)
        self.declare_parameter('motor_to_wheel_ratio', 1.0)
        self.declare_parameter('wheel_track_m', 1.0)
        self.declare_parameter('left_rpm_sign', 1.0)
        self.declare_parameter('right_rpm_sign', 1.0)
        self.declare_parameter('feedback_timeout_s', 0.5)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('max_frames_per_tick', 100)

        self.declare_parameter('feedback_topic', '/stack_can/feedback')
        self.declare_parameter('ground_speed_topic', '/ground_speed_mps')
        self.declare_parameter('yaw_rate_topic', '/wheel_yaw_rate_rad_s')
        self.declare_parameter('left_rpm_topic', '/can/left_motor_rpm')
        self.declare_parameter('right_rpm_topic', '/can/right_motor_rpm')
        self.declare_parameter('left_throttle_topic', '/can/left_throttle_v')
        self.declare_parameter('right_throttle_topic', '/can/right_throttle_v')
        self.declare_parameter('left_can_control_topic', '/can/left_can_control')
        self.declare_parameter('right_can_control_topic', '/can/right_can_control')

        self.enable_can = bool(self.get_parameter('enable_can').value)
        self.can_interface = self.get_parameter('can_interface').value
        self.can_extended = bool(self.get_parameter('can_extended').value)

        self.left_rpm_id = parse_can_id(self.get_parameter('left_rpm_can_id').value, 0x0CF11E05)
        self.right_rpm_id = parse_can_id(self.get_parameter('right_rpm_can_id').value, 0x0CF11E06)
        self.left_throttle_id = parse_can_id(self.get_parameter('left_throttle_can_id').value, 0x0CF1051E)
        self.right_throttle_id = parse_can_id(self.get_parameter('right_throttle_can_id').value, 0x0CF1061E)

        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.motor_to_wheel_ratio = max(1e-6, float(self.get_parameter('motor_to_wheel_ratio').value))
        self.wheel_track_m = max(1e-6, float(self.get_parameter('wheel_track_m').value))
        self.left_rpm_sign = float(self.get_parameter('left_rpm_sign').value)
        self.right_rpm_sign = float(self.get_parameter('right_rpm_sign').value)
        self.feedback_timeout_s = float(self.get_parameter('feedback_timeout_s').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.max_frames_per_tick = int(self.get_parameter('max_frames_per_tick').value)

        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.ground_speed_topic = self.get_parameter('ground_speed_topic').value
        self.yaw_rate_topic = self.get_parameter('yaw_rate_topic').value

        self.pub_feedback = self.create_publisher(CanFeedback, self.feedback_topic, 10)
        self.pub_ground_speed = self.create_publisher(Float32, self.ground_speed_topic, 10)
        self.pub_yaw_rate = self.create_publisher(Float32, self.yaw_rate_topic, 10)
        self.pub_left_rpm = self.create_publisher(Float32, self.get_parameter('left_rpm_topic').value, 10)
        self.pub_right_rpm = self.create_publisher(Float32, self.get_parameter('right_rpm_topic').value, 10)
        self.pub_left_throttle = self.create_publisher(Float32, self.get_parameter('left_throttle_topic').value, 10)
        self.pub_right_throttle = self.create_publisher(Float32, self.get_parameter('right_throttle_topic').value, 10)
        self.pub_left_can_control = self.create_publisher(Bool, self.get_parameter('left_can_control_topic').value, 10)
        self.pub_right_can_control = self.create_publisher(Bool, self.get_parameter('right_can_control_topic').value, 10)

        self.can_bus = None
        if self.enable_can:
            if not HAVE_CAN:
                self.get_logger().error('python-can not installed, CAN feedback disabled')
            else:
                try:
                    self.can_bus = can.Bus(channel=self.can_interface, bustype='socketcan', fd=False)
                    self.get_logger().info(
                        f'CAN feedback enabled on {self.can_interface}: '
                        f'rpm=0x{self.left_rpm_id:X}/0x{self.right_rpm_id:X}, '
                        f'throttle=0x{self.left_throttle_id:X}/0x{self.right_throttle_id:X}'
                    )
                except Exception as e:
                    self.get_logger().error(f'Open CAN feedback bus failed: {e}')
                    self.can_bus = None

        self.left_motor_rpm: Optional[float] = None
        self.right_motor_rpm: Optional[float] = None
        self.left_throttle_raw: Optional[int] = None
        self.right_throttle_raw: Optional[int] = None
        self.left_control_mode: Optional[int] = None
        self.right_control_mode: Optional[int] = None

        self.t_left_rpm: Optional[float] = None
        self.t_right_rpm: Optional[float] = None
        self.t_left_throttle: Optional[float] = None
        self.t_right_throttle: Optional[float] = None

        period = 1.0 / max(1.0, publish_rate_hz)
        self.create_timer(period, self.on_timer)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def valid_since(self, stamp_sec: Optional[float], now_sec: float) -> bool:
        if stamp_sec is None:
            return False
        return (now_sec - stamp_sec) <= self.feedback_timeout_s

    def read_can_frames(self):
        if not (self.enable_can and self.can_bus):
            return

        now_sec = self.now_sec()
        for _ in range(max(1, self.max_frames_per_tick)):
            try:
                frame = self.can_bus.recv(timeout=0.0)
            except Exception as e:
                self.get_logger().warn(f'CAN feedback receive failed: {e}')
                return

            if frame is None:
                return

            if self.can_extended and not frame.is_extended_id:
                continue

            can_id = int(frame.arbitration_id)
            data = bytes(frame.data)

            if can_id == self.left_rpm_id:
                rpm = self.decode_rpm(data)
                if rpm is not None:
                    self.left_motor_rpm = rpm
                    self.t_left_rpm = now_sec
            elif can_id == self.right_rpm_id:
                rpm = self.decode_rpm(data)
                if rpm is not None:
                    self.right_motor_rpm = rpm
                    self.t_right_rpm = now_sec
            elif can_id == self.left_throttle_id:
                parsed = self.decode_throttle(data)
                if parsed is not None:
                    self.left_throttle_raw, self.left_control_mode = parsed
                    self.t_left_throttle = now_sec
            elif can_id == self.right_throttle_id:
                parsed = self.decode_throttle(data)
                if parsed is not None:
                    self.right_throttle_raw, self.right_control_mode = parsed
                    self.t_right_throttle = now_sec

    @staticmethod
    def decode_rpm(data: bytes) -> Optional[float]:
        if len(data) < 2:
            return None
        return float(int(data[0]) + int(data[1]) * 256)

    @staticmethod
    def decode_throttle(data: bytes) -> Optional[Tuple[int, int]]:
        if len(data) < 2:
            return None
        return int(data[0]), int(data[1])

    def rpm_to_mps(self, rpm: float, sign: float) -> float:
        wheel_rpm = (rpm * sign) / self.motor_to_wheel_ratio
        return wheel_rpm * (2.0 * math.pi * self.wheel_radius_m) / 60.0

    @staticmethod
    def throttle_raw_to_v(raw: Optional[int]) -> float:
        if raw is None:
            return 0.0
        return float(raw) * 5.0 / 255.0

    def on_timer(self):
        self.read_can_frames()

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        rpm_left_valid = self.valid_since(self.t_left_rpm, now_sec)
        rpm_right_valid = self.valid_since(self.t_right_rpm, now_sec)
        throttle_left_valid = self.valid_since(self.t_left_throttle, now_sec)
        throttle_right_valid = self.valid_since(self.t_right_throttle, now_sec)

        left_rpm = self.left_motor_rpm if rpm_left_valid and self.left_motor_rpm is not None else 0.0
        right_rpm = self.right_motor_rpm if rpm_right_valid and self.right_motor_rpm is not None else 0.0
        left_mps = self.rpm_to_mps(left_rpm, self.left_rpm_sign) if rpm_left_valid else 0.0
        right_mps = self.rpm_to_mps(right_rpm, self.right_rpm_sign) if rpm_right_valid else 0.0

        if rpm_left_valid and rpm_right_valid:
            ground_speed_mps = 0.5 * (left_mps + right_mps)
            avg_motor_rpm = 0.5 * (left_rpm + right_rpm)
        elif rpm_left_valid:
            ground_speed_mps = left_mps
            avg_motor_rpm = left_rpm
        elif rpm_right_valid:
            ground_speed_mps = right_mps
            avg_motor_rpm = right_rpm
        else:
            ground_speed_mps = 0.0
            avg_motor_rpm = 0.0

        yaw_rate_rad_s = (right_mps - left_mps) / self.wheel_track_m if (rpm_left_valid and rpm_right_valid) else 0.0

        feedback = CanFeedback()
        feedback.stamp = now.to_msg()
        feedback.rpm_left_valid = rpm_left_valid
        feedback.rpm_right_valid = rpm_right_valid
        feedback.throttle_left_valid = throttle_left_valid
        feedback.throttle_right_valid = throttle_right_valid
        feedback.left_rpm_id = self.left_rpm_id
        feedback.right_rpm_id = self.right_rpm_id
        feedback.left_throttle_id = self.left_throttle_id
        feedback.right_throttle_id = self.right_throttle_id
        feedback.left_motor_rpm = float(left_rpm)
        feedback.right_motor_rpm = float(right_rpm)
        feedback.avg_motor_rpm = float(avg_motor_rpm)
        feedback.left_wheel_speed_mps = float(left_mps)
        feedback.right_wheel_speed_mps = float(right_mps)
        feedback.ground_speed_mps = float(ground_speed_mps)
        feedback.yaw_rate_rad_s = float(yaw_rate_rad_s)
        feedback.left_throttle_raw = int(self.left_throttle_raw or 0)
        feedback.right_throttle_raw = int(self.right_throttle_raw or 0)
        feedback.left_throttle_v = self.throttle_raw_to_v(self.left_throttle_raw) if throttle_left_valid else 0.0
        feedback.right_throttle_v = self.throttle_raw_to_v(self.right_throttle_raw) if throttle_right_valid else 0.0
        feedback.left_control_mode = int(self.left_control_mode or 0)
        feedback.right_control_mode = int(self.right_control_mode or 0)
        feedback.left_can_control = bool(throttle_left_valid and self.left_control_mode == 1)
        feedback.right_can_control = bool(throttle_right_valid and self.right_control_mode == 1)

        self.pub_feedback.publish(feedback)

        msg = Float32()
        msg.data = float(ground_speed_mps)
        self.pub_ground_speed.publish(msg)

        msg = Float32()
        msg.data = float(yaw_rate_rad_s)
        self.pub_yaw_rate.publish(msg)

        msg = Float32()
        msg.data = float(left_rpm)
        self.pub_left_rpm.publish(msg)

        msg = Float32()
        msg.data = float(right_rpm)
        self.pub_right_rpm.publish(msg)

        msg = Float32()
        msg.data = feedback.left_throttle_v
        self.pub_left_throttle.publish(msg)

        msg = Float32()
        msg.data = feedback.right_throttle_v
        self.pub_right_throttle.publish(msg)

        bmsg = Bool()
        bmsg.data = feedback.left_can_control
        self.pub_left_can_control.publish(bmsg)

        bmsg = Bool()
        bmsg.data = feedback.right_can_control
        self.pub_right_can_control.publish(bmsg)


def main(args=None):
    rclpy.init(args=args)
    node = CanFeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
