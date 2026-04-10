#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import Float32, Float32MultiArray, Bool


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic')

        # 参数
        self.declare_parameter('port', '/dev/ttyTHS0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('alpha', 0.3)               # EMA 平滑系数
        self.declare_parameter('near_threshold_m', 0.50)   # 近距阈值
        self.declare_parameter('jump_reject_m', 1.0)       # 跳变抑制阈值
        self.declare_parameter('enable_jump_reject', True)

        port = self.get_parameter('port').value
        baudrate = int(self.get_parameter('baudrate').value)
        self.alpha = float(self.get_parameter('alpha').value)
        self.near_threshold_m = float(self.get_parameter('near_threshold_m').value)
        self.jump_reject_m = float(self.get_parameter('jump_reject_m').value)
        self.enable_jump_reject = bool(self.get_parameter('enable_jump_reject').value)

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f'Serial opened: {port} @ {baudrate}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial: {e}')
            raise

        # 发布器
        self.pub_all = self.create_publisher(Float32MultiArray, 'ultrasonic_distances', 10)
        self.pub_front_left = self.create_publisher(Float32, 'ultrasonic/front_left', 10)
        self.pub_front_right = self.create_publisher(Float32, 'ultrasonic/front_right', 10)
        self.pub_rear_left = self.create_publisher(Float32, 'ultrasonic/rear_left', 10)
        self.pub_rear_right = self.create_publisher(Float32, 'ultrasonic/rear_right', 10)
        self.pub_obstacle_near = self.create_publisher(Bool, 'ultrasonic_obstacle_near', 10)

        self.timer = self.create_timer(0.01, self.read_serial)

        self.buffer = bytearray()
        self.filtered_ranges = [None, None, None, None]
        self.last_raw_ranges = [None, None, None, None]

    def read_serial(self):
        try:
            data = self.ser.read(64)
            if data:
                self.buffer.extend(data)
                self.parse_buffer()
        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')

    def parse_buffer(self):
        frame_len = 10

        while len(self.buffer) >= frame_len:
            if self.buffer[0] != 0xFF:
                self.buffer.pop(0)
                continue

            if len(self.buffer) < frame_len:
                return

            frame = self.buffer[:frame_len]

            checksum = sum(frame[0:9]) & 0xFF
            if checksum != frame[9]:
                self.get_logger().warn(
                    f'Checksum failed: calc=0x{checksum:02X}, recv=0x{frame[9]:02X}'
                )
                self.buffer.pop(0)
                continue

            # 协议: 0xFF + 8字节数据 + 1字节校验
            raw_m = [
                ((frame[1] << 8) | frame[2]) / 1000.0,
                ((frame[3] << 8) | frame[4]) / 1000.0,
                ((frame[5] << 8) | frame[6]) / 1000.0,
                ((frame[7] << 8) | frame[8]) / 1000.0,
            ]

            smoothed_m = []
            for i in range(4):
                x = raw_m[i]

                if self.enable_jump_reject and self.last_raw_ranges[i] is not None:
                    if abs(x - self.last_raw_ranges[i]) > self.jump_reject_m:
                        self.get_logger().warn(
                            f'Jump rejected on sensor {i+1}: '
                            f'last={self.last_raw_ranges[i]:.3f} m, raw={x:.3f} m'
                        )
                        x = self.last_raw_ranges[i]

                self.last_raw_ranges[i] = x

                if self.filtered_ranges[i] is None:
                    self.filtered_ranges[i] = x
                else:
                    self.filtered_ranges[i] = (
                        self.alpha * x + (1.0 - self.alpha) * self.filtered_ranges[i]
                    )

                smoothed_m.append(float(self.filtered_ranges[i]))

            self.publish_data(smoothed_m)
            self.buffer = self.buffer[frame_len:]

    def publish_data(self, values_m):
        msg_all = Float32MultiArray()
        msg_all.data = values_m
        self.pub_all.publish(msg_all)

        msg = Float32()

        msg.data = values_m[0]
        self.pub_front_left.publish(msg)

        msg = Float32()
        msg.data = values_m[1]
        self.pub_front_right.publish(msg)

        msg = Float32()
        msg.data = values_m[2]
        self.pub_rear_left.publish(msg)

        msg = Float32()
        msg.data = values_m[3]
        self.pub_rear_right.publish(msg)

        near = any(v > 0.0 and v < self.near_threshold_m for v in values_m)
        msg_near = Bool()
        msg_near.data = near
        self.pub_obstacle_near.publish(msg_near)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
