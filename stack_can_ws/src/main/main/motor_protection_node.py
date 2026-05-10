#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from stack_msgs.msg import CanFeedback


class MotorProtectionNode(Node):
    """
    Motor stall protection node.

    The node watches CAN feedback from can_feedback_node. If a motor receives
    continuous throttle output while its rpm remains very low, it publishes
    /abort=True. stack_can_executor already latches /abort into ESTOP.
    """

    def __init__(self):
        super().__init__('motor_protection_node')

        self.declare_parameter('feedback_topic', '/stack_can/feedback')
        self.declare_parameter('abort_topic', '/abort')

        self.declare_parameter('throttle_high_v', 0.8)
        self.declare_parameter('stall_rpm_threshold', 30.0)
        self.declare_parameter('stall_timeout_s', 1.0)
        self.declare_parameter('abort_on_feedback_invalid', False)

        self.declare_parameter('protect_left_motor', True)
        self.declare_parameter('protect_right_motor', True)
        self.declare_parameter('latch_abort', True)

        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.abort_topic = self.get_parameter('abort_topic').value

        self.throttle_high_v = float(self.get_parameter('throttle_high_v').value)
        self.stall_rpm_threshold = float(self.get_parameter('stall_rpm_threshold').value)
        self.stall_timeout_s = float(self.get_parameter('stall_timeout_s').value)
        self.abort_on_feedback_invalid = bool(self.get_parameter('abort_on_feedback_invalid').value)

        self.protect_left_motor = bool(self.get_parameter('protect_left_motor').value)
        self.protect_right_motor = bool(self.get_parameter('protect_right_motor').value)
        self.latch_abort = bool(self.get_parameter('latch_abort').value)

        self.abort_latched = False
        self.left_stall_start_sec = None
        self.right_stall_start_sec = None

        self.pub_abort = self.create_publisher(Bool, self.abort_topic, 10)
        self.create_subscription(CanFeedback, self.feedback_topic, self.on_feedback, 10)

        self.get_logger().info(
            'MotorProtectionNode started: '
            f'feedback_topic={self.feedback_topic}, '
            f'abort_topic={self.abort_topic}, '
            f'throttle_high_v={self.throttle_high_v:.2f}, '
            f'stall_rpm_threshold={self.stall_rpm_threshold:.1f}, '
            f'stall_timeout_s={self.stall_timeout_s:.2f}, '
            f'abort_on_feedback_invalid={self.abort_on_feedback_invalid}'
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_abort(self, reason: str):
        if self.latch_abort and self.abort_latched:
            return

        self.abort_latched = True

        msg = Bool()
        msg.data = True
        self.pub_abort.publish(msg)

        self.get_logger().error(f'MOTOR PROTECTION ABORT: {reason}')

    def update_stall_state(self, side: str, stall_now: bool, reason: str):
        now = self.now_sec()

        if side == 'left':
            if stall_now:
                if self.left_stall_start_sec is None:
                    self.left_stall_start_sec = now
                    self.get_logger().warn(f'left motor possible stall started: {reason}')
                    return

                duration = now - self.left_stall_start_sec
                if duration >= self.stall_timeout_s:
                    self.publish_abort(f'{reason}, duration={duration:.2f}s')
            else:
                self.left_stall_start_sec = None

        elif side == 'right':
            if stall_now:
                if self.right_stall_start_sec is None:
                    self.right_stall_start_sec = now
                    self.get_logger().warn(f'right motor possible stall started: {reason}')
                    return

                duration = now - self.right_stall_start_sec
                if duration >= self.stall_timeout_s:
                    self.publish_abort(f'{reason}, duration={duration:.2f}s')
            else:
                self.right_stall_start_sec = None

    def check_left_motor(self, msg: CanFeedback):
        feedback_valid = bool(msg.rpm_left_valid and msg.throttle_left_valid)

        if not feedback_valid:
            self.left_stall_start_sec = None
            if self.abort_on_feedback_invalid:
                self.publish_abort(
                    'left motor feedback invalid: '
                    f'rpm_left_valid={msg.rpm_left_valid}, '
                    f'throttle_left_valid={msg.throttle_left_valid}'
                )
            return

        high_throttle = abs(float(msg.left_throttle_v)) >= self.throttle_high_v
        low_rpm = abs(float(msg.left_motor_rpm)) <= self.stall_rpm_threshold
        can_control = bool(msg.left_can_control)

        stall_now = can_control and high_throttle and low_rpm
        reason = (
            'left motor stall detected: '
            f'throttle={msg.left_throttle_v:.2f}V, '
            f'rpm={msg.left_motor_rpm:.1f}, '
            f'can_control={msg.left_can_control}'
        )
        self.update_stall_state('left', stall_now, reason)

    def check_right_motor(self, msg: CanFeedback):
        feedback_valid = bool(msg.rpm_right_valid and msg.throttle_right_valid)

        if not feedback_valid:
            self.right_stall_start_sec = None
            if self.abort_on_feedback_invalid:
                self.publish_abort(
                    'right motor feedback invalid: '
                    f'rpm_right_valid={msg.rpm_right_valid}, '
                    f'throttle_right_valid={msg.throttle_right_valid}'
                )
            return

        high_throttle = abs(float(msg.right_throttle_v)) >= self.throttle_high_v
        low_rpm = abs(float(msg.right_motor_rpm)) <= self.stall_rpm_threshold
        can_control = bool(msg.right_can_control)

        stall_now = can_control and high_throttle and low_rpm
        reason = (
            'right motor stall detected: '
            f'throttle={msg.right_throttle_v:.2f}V, '
            f'rpm={msg.right_motor_rpm:.1f}, '
            f'can_control={msg.right_can_control}'
        )
        self.update_stall_state('right', stall_now, reason)

    def on_feedback(self, msg: CanFeedback):
        if self.latch_abort and self.abort_latched:
            return

        if self.protect_left_motor:
            self.check_left_motor(msg)

        if self.protect_right_motor:
            self.check_right_motor(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorProtectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
