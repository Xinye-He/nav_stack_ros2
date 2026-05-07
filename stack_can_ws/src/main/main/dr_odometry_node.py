#!/usr/bin/env python3
# coding: utf-8
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def heading_north_cw_deg_to_yaw_enu_rad(hdg_deg: float) -> float:
    return wrap_pi(math.radians(90.0 - hdg_deg))


class DROdometry(Node):
    def __init__(self):
        super().__init__('dr_odometry')

        self.declare_parameter('speed_topic', '/ground_speed_mps')
        self.declare_parameter('heading_topic', '/vehicle_heading_deg')
        self.declare_parameter('yaw_rate_topic', '/wheel_yaw_rate_rad_s')
        self.declare_parameter('odom_topic', '/dr/odom')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('use_heading', True)
        self.declare_parameter('use_yaw_rate', True)
        self.declare_parameter('heading_timeout_s', 0.5)
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('fallback_speed', 0.0)  # If no speed data, use this value (m/s)

        self.speed_topic = self.get_parameter('speed_topic').value
        self.heading_topic = self.get_parameter('heading_topic').value
        self.yaw_rate_topic = self.get_parameter('yaw_rate_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.use_heading = bool(self.get_parameter('use_heading').value)
        self.use_yaw_rate = bool(self.get_parameter('use_yaw_rate').value)
        self.heading_timeout_s = float(self.get_parameter('heading_timeout_s').value)
        self.yaw_offset = math.radians(float(self.get_parameter('yaw_offset_deg').value))
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.fallback_speed = float(self.get_parameter('fallback_speed').value)
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.dt_timer = 1.0 / max(1.0, rate_hz)

        self.first_publish = True  # For "only once" option

        self.sub_v = self.create_subscription(Float32, self.speed_topic, self.on_speed, qos_profile_sensor_data)
        self.sub_h = None
        if self.use_heading:
            self.sub_h = self.create_subscription(Float32, self.heading_topic, self.on_heading, qos_profile_sensor_data)
        self.sub_yaw_rate = None
        if self.use_yaw_rate:
            self.sub_yaw_rate = self.create_subscription(Float32, self.yaw_rate_topic, self.on_yaw_rate, qos_profile_sensor_data)

        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tfb = TransformBroadcaster(self) if self.publish_tf else None

        self.v_mps: Optional[float] = None
        self.yaw_rate: Optional[float] = None
        self.yaw_meas: Optional[float] = None

        self.x = 0.0
        self.y = 0.0
        self.yaw_cont: Optional[float] = None
        self.last_yaw_meas: Optional[float] = None
        self.last_heading_time: Optional[float] = None

        self.last_time = self.get_clock().now()
        self.create_timer(self.dt_timer, self.on_timer)

    def on_speed(self, msg: Float32):
        v = float(msg.data)
        if math.isfinite(v):
            self.v_mps = v
            self.get_logger().debug(f"Received speed: {v} m/s")
        else:
            self.get_logger().warn("Received non-finite speed data; using fallback")

    def on_heading(self, msg: Float32):
        hdg = float(msg.data)
        if not math.isfinite(hdg):
            self.get_logger().warn("Received non-finite heading data")
            return
        yaw = heading_north_cw_deg_to_yaw_enu_rad(hdg)
        yaw = wrap_pi(yaw + self.yaw_offset)
        self.yaw_meas = yaw
        self.get_logger().debug(f"Received heading: {hdg} deg, converted to yaw: {yaw} rad")
        self.last_heading_time = self.get_clock().now().nanoseconds * 1e-9

        if self.last_yaw_meas is None:
            self.last_yaw_meas = yaw
            self.yaw_cont = yaw
        else:
            dy = wrap_pi(yaw - self.last_yaw_meas)
            self.last_yaw_meas = yaw
            self.yaw_cont = (self.yaw_cont + dy) if (self.yaw_cont is not None) else yaw

    def on_yaw_rate(self, msg: Float32):
        wz = float(msg.data)
        if math.isfinite(wz):
            self.yaw_rate = wz
        else:
            self.get_logger().warn("Received non-finite yaw-rate data")

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-3
        self.last_time = now

        v = self.v_mps if self.v_mps is not None else self.fallback_speed
        if self.yaw_cont is None:
            if self.use_yaw_rate and self.yaw_rate is not None:
                self.yaw_cont = 0.0
                self.get_logger().warn("No heading data; initialized DR yaw from 0 and using wheel yaw-rate")
            else:
                self.get_logger().warn("No heading/yaw-rate data; not publishing odom -> base_link")
                return

        now_sec = now.nanoseconds * 1e-9
        heading_fresh = (
            self.use_heading and
            self.last_heading_time is not None and
            (now_sec - self.last_heading_time) <= self.heading_timeout_s
        )
        if self.use_yaw_rate and self.yaw_rate is not None and not heading_fresh:
            self.yaw_cont += self.yaw_rate * dt

        self.x += v * math.cos(self.yaw_cont) * dt
        self.y += v * math.sin(self.yaw_cont) * dt

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.yaw_cont)

        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)
        odom.twist.twist.linear.x = float(v)
        if self.yaw_rate is not None:
            odom.twist.twist.angular.z = float(self.yaw_rate)
        self.pub_odom.publish(odom)

        if self.tfb is not None and self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = float(qx)
            t.transform.rotation.y = float(qy)
            t.transform.rotation.z = float(qz)
            t.transform.rotation.w = float(qw)
            self.tfb.sendTransform(t)
            if self.first_publish:  # 只在第一次输出
                self.get_logger().info("Successfully published odom -> base_link TF for the first time")
                self.first_publish = False  # 后续不再输出

def main():
    rclpy.init()
    node = DROdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
