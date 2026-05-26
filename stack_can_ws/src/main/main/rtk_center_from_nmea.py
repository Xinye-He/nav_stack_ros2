#!/usr/bin/env python3
# coding: utf-8
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


# WGS84 constants (semi-major axis a and flattening f)
# Source: DoD/NIMA WGS 84 definition (e.g., NIMA TR8350.2)  [WGS84: a=6378137, f=1/298.257223563]
A_WGS84 = 6378137.0
F_WGS84 = 1.0 / 298.257223563
E2_WGS84 = F_WGS84 * (2.0 - F_WGS84)


def wrap_2pi(rad: float) -> float:
    rad = rad % (2.0 * math.pi)
    return rad


def heading_north_cw_deg_to_yaw_east_ccw_rad(heading_deg: float) -> float:
    """
    Convert heading in degrees (0=N, 90=E, clockwise positive)
    to yaw in radians (0=+X East, CCW positive) in ENU.
    """
    h = math.radians(heading_deg)
    yaw = (math.pi / 2.0) - h
    return wrap_2pi(yaw)


def radii_of_curvature(lat_rad: float):
    """
    Returns (M, N) at latitude:
      M: meridian radius of curvature
      N: prime vertical radius of curvature
    """
    s = math.sin(lat_rad)
    denom = math.sqrt(1.0 - E2_WGS84 * s * s)
    N = A_WGS84 / denom
    M = A_WGS84 * (1.0 - E2_WGS84) / (denom ** 3)
    return M, N


class RtkCenterFromNmea(Node):
    def __init__(self):
        super().__init__('rtk_center_from_nmea')

        # ---- parameters ----
        self.declare_parameter('fix_topic', '/fix')
        self.declare_parameter('heading_topic', '/heading_deg')

        self.declare_parameter('out_fix_center_topic', '/fix_center')
        self.declare_parameter('out_odom_center_topic', '/odom_center')
        self.declare_parameter('out_vehicle_heading_topic', '/vehicle_heading_deg')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # main antenna -> vehicle center (meters) expressed in base_link frame (x forward, y left)
        self.declare_parameter('main_to_center_x', -1.80)
        self.declare_parameter('main_to_center_y', -0.75)
        self.declare_parameter('main_to_center_z', 0.0)

        # heading interpretation/calibration
        self.declare_parameter('heading_from_north_cw', True)  # THS typical
        self.declare_parameter('heading_offset_deg', 0.0)      # small calibration if needed

        # baseline (main->secondary) to vehicle forward yaw offset, in degrees, applied in yaw_east_ccw space
        # For "main on left, secondary on right": baseline points to vehicle right (-Y), so vehicle forward = baseline + 90deg
        self.declare_parameter('baseline_to_vehicle_yaw_offset_deg', 90.0)

        # output
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('set_origin_on_first_center_fix', True)

        self.fix_topic = self.get_parameter('fix_topic').value
        self.heading_topic = self.get_parameter('heading_topic').value

        self.out_fix_center_topic = self.get_parameter('out_fix_center_topic').value
        self.out_odom_center_topic = self.get_parameter('out_odom_center_topic').value
        self.out_vehicle_heading_topic = self.get_parameter('out_vehicle_heading_topic').value

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.dx = float(self.get_parameter('main_to_center_x').value)
        self.dy = float(self.get_parameter('main_to_center_y').value)
        self.dz = float(self.get_parameter('main_to_center_z').value)

        self.heading_from_north_cw = bool(self.get_parameter('heading_from_north_cw').value)
        self.heading_offset_deg = float(self.get_parameter('heading_offset_deg').value)
        self.baseline_to_vehicle_offset_deg = float(self.get_parameter('baseline_to_vehicle_yaw_offset_deg').value)

        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.set_origin_on_first_center_fix = bool(self.get_parameter('set_origin_on_first_center_fix').value)

        # ---- pubs/subs ----
        self.sub_fix = self.create_subscription(NavSatFix, self.fix_topic, self.on_fix, qos_profile_sensor_data)
        self.sub_heading = self.create_subscription(Float32, self.heading_topic, self.on_heading, qos_profile_sensor_data)

        self.pub_fix_center = self.create_publisher(NavSatFix, self.out_fix_center_topic, 10)
        self.pub_odom_center = self.create_publisher(Odometry, self.out_odom_center_topic, 10)
        self.pub_vehicle_heading = self.create_publisher(Float32, self.out_vehicle_heading_topic, 10)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # ---- state ----
        self.last_fix: Optional[NavSatFix] = None
        self.last_heading_deg: Optional[float] = None

        # local ENU origin
        self.origin_set = False
        self.lat0_rad = 0.0
        self.lon0_rad = 0.0
        self.M0 = 0.0
        self.N0 = 0.0

    def on_fix(self, msg: NavSatFix):
        # Basic validity check
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        self.last_fix = msg
        self.try_publish()

    def on_heading(self, msg: Float32):
        hd = float(msg.data)
        if not math.isfinite(hd):
            return
        # allow 0..360, also tolerate negatives by wrap
        self.last_heading_deg = hd
        self.try_publish()

    def compute_vehicle_yaw(self, heading_deg: float) -> float:
        """
        heading_deg is baseline heading (main->secondary) by your description.
        Returns vehicle yaw in ENU (0=East, CCW positive).
        """
        heading_deg = (heading_deg + self.heading_offset_deg) % 360.0

        if self.heading_from_north_cw:
            yaw_baseline = heading_north_cw_deg_to_yaw_east_ccw_rad(heading_deg)
        else:
            # if heading is already yaw (0=East, CCW), just convert
            yaw_baseline = wrap_2pi(math.radians(heading_deg))

        yaw_vehicle = wrap_2pi(yaw_baseline + math.radians(self.baseline_to_vehicle_offset_deg))
        return yaw_vehicle

    def rotate_base_to_enu(self, yaw_vehicle: float, x: float, y: float):
        """
        Rotate a vector from base_link (x fwd, y left) into ENU (x East, y North),
        using yaw_vehicle defined in ENU (0 East, CCW positive).
        """
        c = math.cos(yaw_vehicle)
        s = math.sin(yaw_vehicle)
        east = c * x - s * y
        north = s * x + c * y
        return east, north

    def enu_to_latlon_delta(self, lat_rad: float, east_m: float, north_m: float):
        """
        Convert small ENU offsets (east, north in meters) to delta lat/lon in radians at given latitude.
        Uses radii of curvature M, N for WGS84.
        """
        M, N = radii_of_curvature(lat_rad)
        dlat = north_m / M
        dlon = east_m / (N * math.cos(lat_rad))
        return dlat, dlon

    def latlon_to_local_enu(self, lat_rad: float, lon_rad: float):
        """
        Convert lat/lon to local ENU (east,north) meters w.r.t origin (lat0,lon0).
        Uses M0,N0 at origin for small-area navigation.
        """
        dlat = lat_rad - self.lat0_rad
        dlon = lon_rad - self.lon0_rad
        east = dlon * (self.N0 * math.cos(self.lat0_rad))
        north = dlat * self.M0
        return east, north

    def try_publish(self):
        if self.last_fix is None or self.last_heading_deg is None:
            return

        fix = self.last_fix
        heading_deg = self.last_heading_deg

        lat_rad = math.radians(fix.latitude)
        lon_rad = math.radians(fix.longitude)

        yaw_vehicle = self.compute_vehicle_yaw(heading_deg)

        # main->center offset in ENU
        off_e, off_n = self.rotate_base_to_enu(yaw_vehicle, self.dx, self.dy)

        # center lat/lon
        dlat, dlon = self.enu_to_latlon_delta(lat_rad, off_e, off_n)
        lat_c = lat_rad + dlat
        lon_c = lon_rad + dlon

        lat_c_deg = math.degrees(lat_c)
        lon_c_deg = math.degrees(lon_c)

        # publish center fix
        fix_c = NavSatFix()
        fix_c.header.stamp = self.get_clock().now().to_msg()
        fix_c.header.frame_id = self.base_frame  # “这个fix表示车身中心”
        fix_c.status = fix.status
        fix_c.latitude = float(lat_c_deg)
        fix_c.longitude = float(lon_c_deg)
        fix_c.altitude = float(fix.altitude) if math.isfinite(fix.altitude) else float('nan')
        fix_c.position_covariance = fix.position_covariance
        fix_c.position_covariance_type = fix.position_covariance_type
        self.pub_fix_center.publish(fix_c)

        # publish vehicle heading (north-cw) for convenience
        # heading_north_cw = 90deg - yaw_east_ccw
        yaw_deg = math.degrees(yaw_vehicle)
        vehicle_heading_deg = (90.0 - yaw_deg) % 360.0
        hmsg = Float32()
        hmsg.data = float(vehicle_heading_deg)
        self.pub_vehicle_heading.publish(hmsg)

        # set local origin at first center fix (optional)
        if self.set_origin_on_first_center_fix and (not self.origin_set):
            self.lat0_rad = lat_c
            self.lon0_rad = lon_c
            self.M0, self.N0 = radii_of_curvature(self.lat0_rad)
            self.origin_set = True

        if not self.origin_set:
            return

        # local ENU odom of center
        e, n = self.latlon_to_local_enu(lat_c, lon_c)
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw_vehicle)

        odom = Odometry()
        odom.header.stamp = fix_c.header.stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(e)   # x=East
        odom.pose.pose.position.y = float(n)   # y=North
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)
        self.pub_odom_center.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(e)
            t.transform.translation.y = float(n)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = float(qx)
            t.transform.rotation.y = float(qy)
            t.transform.rotation.z = float(qz)
            t.transform.rotation.w = float(qw)
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = RtkCenterFromNmea()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
