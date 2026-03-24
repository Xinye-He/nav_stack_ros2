#!/usr/bin/env python3
import math
import csv
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time

from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from std_msgs.msg import Bool, Float32, UInt8
from nav_msgs.msg import Path

import tf2_ros

from stack_msgs.msg import StackCommand   # 自定义消息


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    half = 0.5 * yaw
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def heading_csv_deg_to_enu_rad(hdg_deg: float) -> float:
    # 输入：0°=北，90°=东，顺时针为正 -> 输出 ENU: 0=东，逆时针为正
    return wrap_pi(math.radians(90.0 - hdg_deg))


class LLA2ENU:
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2 - f)

    def __init__(self, lat0_deg: float, lon0_deg: float, alt0: float = 0.0):
        self.lat0 = math.radians(lat0_deg)
        self.lon0 = math.radians(lon0_deg)
        self.alt0 = alt0
        self.x0, self.y0, self.z0 = self.lla_to_ecef(lat0_deg, lon0_deg, alt0)

        sphi = math.sin(self.lat0)
        cphi = math.cos(self.lat0)
        slam = math.sin(self.lon0)
        clam = math.cos(self.lon0)
        self.Re = [
            [-slam,            clam,           0.0],
            [-sphi*clam, -sphi*slam,  cphi],
            [ cphi*clam,  cphi*slam,  sphi]
        ]

    @classmethod
    def lla_to_ecef(cls, lat_deg, lon_deg, alt):
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        sphi = math.sin(lat)
        cphi = math.cos(lat)
        slam = math.sin(lon)
        clam = math.cos(lon)
        N = cls.a / math.sqrt(1.0 - cls.e2 * sphi * sphi)
        x = (N + alt) * cphi * clam
        y = (N + alt) * cphi * slam
        z = (N * (1.0 - cls.e2) + alt) * sphi
        return x, y, z

    def lla_to_enu(self, lat_deg, lon_deg, alt=0.0):
        x, y, z = self.lla_to_ecef(lat_deg, lon_deg, alt)
        dx = x - self.x0
        dy = y - self.y0
        dz = z - self.z0
        e = self.Re[0][0]*dx + self.Re[0][1]*dy + self.Re[0][2]*dz
        n = self.Re[1][0]*dx + self.Re[1][1]*dy + self.Re[1][2]*dz
        u = self.Re[2][0]*dx + self.Re[2][1]*dy + self.Re[2][2]*dz
        return e, n, u


class TrajWaypointFollower(Node):
    """
    只负责：
      - 路径跟随
      - 任务点等待/退出
      - 基于误差选择离散档位 pre_speed_kmh + angle_deg
    输出:
      - /stack_cmd/traj (StackCommand)
    不再直接发 CAN。
    """
    DS_PAUSED = 0
    DS_RUNNING = 1
    DS_ESTOP  = 2

    def __init__(self):
        super().__init__('traj_waypoint_follower')

        # I/O topics
        self.declare_parameter('path_csv', '')
        self.declare_parameter('gps_topic', '/fix')
        self.declare_parameter('imu_topic', '/imu/data')

        # 路段/切换
        self.declare_parameter('advance_when_close', 2.5)
        self.declare_parameter('t_advance_min', 0.9)
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('auto_align_yaw', False)

        # Heading policy
        self.declare_parameter('use_csv_heading', True)
        self.declare_parameter('align_heading_only_at_task_points', True)
        self.declare_parameter('heading_align_dist', 1.0)

        # Task point handling
        self.declare_parameter('wp_reached_dist', 0.8)
        self.declare_parameter('wp_heading_tol_deg', 10.0)
        self.declare_parameter('stop_turn_tol_deg', 5.0)
        self.declare_parameter('wait_for_task_done', True)
        self.declare_parameter('task_done_topic', '/task_done')

        # Lookahead
        self.declare_parameter('lookahead_dist', 1.5)

        # Frames & viz
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('traj_path_len', 2000)

        # DR parameters
        self.declare_parameter('dead_reckon', True)
        self.declare_parameter('dr_alpha', 0.6)
        self.declare_parameter('dr_use_gps_speed', True)

        # Heading source (RTK)
        self.declare_parameter('use_rtk_heading', True)
        self.declare_parameter('rtk_heading_topic', '/gps/heading_deg')

        # 离散速度/角度参数（基于你 stack 的实际档位）
        self.declare_parameter('vcu_speed_fast_kmh', 8.0)
        self.declare_parameter('vcu_speed_slow_kmh', 4.0)
        self.declare_parameter('vcu_speed_stop_kmh', 0.0)

        self.declare_parameter('vcu_angle_move_limit_deg', 20.0)
        self.declare_parameter('vcu_turn_small_cmd_deg', 10.0)
        self.declare_parameter('vcu_turn_large_cmd_deg', 20.0)
        self.declare_parameter('vcu_turn_small_thresh_deg', 10.0)
        self.declare_parameter('vcu_turn_large_thresh_deg', 25.0)

        self.declare_parameter('vcu_angle_spin_enter_deg', 22.0)
        self.declare_parameter('vcu_angle_spin_exit_deg', 18.0)
        self.declare_parameter('vcu_angle_spin_cmd_deg', 20.0)

        # 拐角识别 & 误差阈值
        self.declare_parameter('strict_corner_mode', True)
        self.declare_parameter('corner_start_dist', 1.0)
        self.declare_parameter('corner_sharp_deg_strict', 45.0)
        self.declare_parameter('strict_hold_deg', 2.0)

        self.declare_parameter('emergency_spin_hdg_deg', 35.0)
        self.declare_parameter('hdg_deadband_deg', 1.0)
        self.declare_parameter('cte_deadband_m', 0.1)
        self.declare_parameter('sign_hysteresis_deg', 1.0)

        # SPIN 预停止时间（从低速/高速进入原地旋转前保持 0速0角 的时间）
        self.declare_parameter('spin_pre_stop_time_s', 1.0)

        # 读参数
        path_csv = self.get_parameter('path_csv').get_parameter_value().string_value
        if not path_csv:
            self.get_logger().error("path_csv is required.")
            raise SystemExit

        self.path_csv = path_csv
        self.gps_topic = self.get_parameter('gps_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value

        self.advance_when_close = float(self.get_parameter('advance_when_close').value)
        self.t_advance_min = float(self.get_parameter('t_advance_min').value)
        self.yaw_offset = math.radians(float(self.get_parameter('yaw_offset_deg').value))
        self.auto_align_yaw = bool(self.get_parameter('auto_align_yaw').value)

        self.use_csv_heading = bool(self.get_parameter('use_csv_heading').value)
        self.align_heading_only_at_task_points = bool(self.get_parameter('align_heading_only_at_task_points').value)
        self.heading_align_dist = float(self.get_parameter('heading_align_dist').value)

        self.wp_reached_dist = float(self.get_parameter('wp_reached_dist').value)
        self.wp_heading_tol = math.radians(float(self.get_parameter('wp_heading_tol_deg').value))
        # 度 & 弧度两种形式都保存
        self.stop_turn_tol_deg = float(self.get_parameter('stop_turn_tol_deg').value)
        self.stop_turn_tol = math.radians(self.stop_turn_tol_deg)
        self.wait_for_task_done = bool(self.get_parameter('wait_for_task_done').value)
        self.task_done_topic = self.get_parameter('task_done_topic').value

        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.traj_path_len = int(self.get_parameter('traj_path_len').value)

        # DR
        self.dead_reckon = bool(self.get_parameter('dead_reckon').value)
        self.dr_alpha = float(self.get_parameter('dr_alpha').value)
        self.dr_use_gps_speed = bool(self.get_parameter('dr_use_gps_speed').value)
        self.dr_x = None
        self.dr_y = None
        self.last_gps_speed = None

        # RTK heading source
        self.use_rtk_heading = bool(self.get_parameter('use_rtk_heading').value)
        self.rtk_heading_topic = self.get_parameter('rtk_heading_topic').value

        # VCU 离散控制相关参数
        self.vcu_speed_fast_kmh = float(self.get_parameter('vcu_speed_fast_kmh').value)
        self.vcu_speed_slow_kmh = float(self.get_parameter('vcu_speed_slow_kmh').value)
        self.vcu_speed_stop_kmh = float(self.get_parameter('vcu_speed_stop_kmh').value)

        self.vcu_angle_move_limit_deg = float(self.get_parameter('vcu_angle_move_limit_deg').value)
        self.vcu_turn_small_cmd_deg = float(self.get_parameter('vcu_turn_small_cmd_deg').value)
        self.vcu_turn_large_cmd_deg = float(self.get_parameter('vcu_turn_large_cmd_deg').value)
        self.vcu_turn_small_thresh_deg = float(self.get_parameter('vcu_turn_small_thresh_deg').value)
        self.vcu_turn_large_thresh_deg = float(self.get_parameter('vcu_turn_large_thresh_deg').value)

        self.vcu_angle_spin_enter_deg = float(self.get_parameter('vcu_angle_spin_enter_deg').value)
        self.vcu_angle_spin_exit_deg = float(self.get_parameter('vcu_angle_spin_exit_deg').value)
        self.vcu_angle_spin_cmd_deg = float(self.get_parameter('vcu_angle_spin_cmd_deg').value)

        self.strict_corner_mode = bool(self.get_parameter('strict_corner_mode').value)
        self.corner_start_dist = float(self.get_parameter('corner_start_dist').value)
        self.corner_sharp_rad_strict = math.radians(float(self.get_parameter('corner_sharp_deg_strict').value))
        self.strict_hold_deg = float(self.get_parameter('strict_hold_deg').value)

        self.emergency_spin_hdg_deg = float(self.get_parameter('emergency_spin_hdg_deg').value)
        self.hdg_deadband_deg = float(self.get_parameter('hdg_deadband_deg').value)
        self.cte_deadband_m = float(self.get_parameter('cte_deadband_m').value)
        self.sign_hysteresis_deg = float(self.get_parameter('sign_hysteresis_deg').value)
        self.angle_sign = 0

        self.spin_pre_stop_time_s = float(self.get_parameter('spin_pre_stop_time_s').value)

        # SPIN 状态机：IDLE / PRE_STOP / SPIN
        self.spin_state = 'IDLE'
        self.spin_pre_stop_until = 0.0
        self.spin_use_next_seg = False
        self.spin_sign = 0.0

        # waypoints
        llh = self.load_csv(path_csv)
        if len(llh) < 1:
            self.get_logger().error("No valid waypoints in CSV")
            raise SystemExit
        llh.sort(key=lambda x: x[0])

        lat0, lon0 = llh[0][1], llh[0][2]
        self.geo = LLA2ENU(lat0, lon0, 0.0)
        self.waypoints_xy: List[Tuple[float, float, float, int]] = []
        for _, lat, lon, hdg_deg_csv, pt_type in llh:
            x, y, _ = self.geo.lla_to_enu(lat, lon, 0.0)
            yaw_enu = heading_csv_deg_to_enu_rad(hdg_deg_csv)
            self.waypoints_xy.append((x, y, yaw_enu, int(pt_type)))

        # State
        self.cur_x = None
        self.cur_y = None
        self.cur_yaw = None
        self.last_time = self.get_clock().now()

        self.seg_idx = 0
        self.aligned = False

        self.waiting_for_task = False
        self.task_done_latch = False

        self.unload_mode = False
        self.unload_end_time = 0.0
        self.unload_reset_end_time = 0.0
        self.unload_finished_once = False

        # Drive state
        self.drive_state = self.DS_PAUSED

        # Subscriptions
        self.create_subscription(NavSatFix, self.gps_topic, self.on_gps, qos_profile_sensor_data)
        if self.use_rtk_heading:
            self.create_subscription(Float32, self.rtk_heading_topic, self.on_rtk_heading, qos_profile_sensor_data)
        else:
            self.create_subscription(Imu, self.imu_topic, self.on_imu, qos_profile_sensor_data)

        self.create_subscription(Float32, '/gps/ground_speed_mps',
                                 lambda m: setattr(self, 'last_gps_speed', float(m.data)),
                                 qos_profile_sensor_data)
        # 新增：重启路径 / 切换 CSV
        self.create_subscription(Bool, '/restart_path', self.on_restart_path, 1)

        # Commands & task topics
        self.create_subscription(Bool,  self.task_done_topic, self.on_task_done, 1)
        self.create_subscription(UInt8, '/drive_cmd',        self.on_drive_cmd, 1)
        self.create_subscription(Bool,  '/abort',            self.on_abort,     1)

        # 任务等待状态输出
        qos_latched = QoSProfile(depth=1)
        qos_latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.pub_task_wait = self.create_publisher(Bool, '/at_task_waiting', qos_latched)

        # StackCommand 输出（给 CAN 执行节点用）
        self.pub_traj_cmd = self.create_publisher(StackCommand, '/stack_cmd/traj', 1)

        # Viz & TF
        qos_tl = QoSProfile(depth=1)
        qos_tl.history = QoSHistoryPolicy.KEEP_LAST
        qos_tl.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub_global_path = self.create_publisher(Path, 'global_path', qos_tl)
        self.pub_traj_path   = self.create_publisher(Path, 'traj_path', 10)
        self.tfb = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.global_path = Path()
        self.global_path.header.frame_id = self.map_frame
        for (x, y, _, _) in self.waypoints_xy:
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            self.global_path.poses.append(ps)

        self.traj_path = Path()
        self.traj_path.header.frame_id = self.map_frame
        self.max_traj_len = self.traj_path_len

        # Debug: 误差等
        self.pub_dbg = self.create_publisher(Float32, 'traj_debug_dummy', 1)  # 可按需换成数组

        # Timer
        self.create_timer(1.0, self._pub_global_path)
        self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.last_pre_kmh_sent = 0.0

        self.get_logger().info(
            f"Loaded {len(self.waypoints_xy)} waypoints. GPS:{self.gps_topic}, RTK:{self.rtk_heading_topic}"
        )

    def on_restart_path(self, msg: Bool):
        if not bool(msg.data):
            return
        self.get_logger().info("Restart path requested, reloading CSV...")

        try:
            self.reload_path()
            # 重置状态
            self.seg_idx = 0
            self.waiting_for_task = False
            if hasattr(self, 'unload_mode'):
                self.unload_mode = False
                self.unload_end_time = 0.0
                self.unload_reset_end_time = 0.0
                self.unload_finished_once = False
            self.drive_state = self.DS_RUNNING
            self._pub_waiting(False)
            self.get_logger().info("Path reloaded, seg_idx=0, drive_state=RUNNING")
        except Exception as e:
            self.get_logger().error(f"Reload path failed: {e}")

    def reload_path(self):
        """
        从 self.path_csv 重新读取轨迹点，重建 self.geo / self.waypoints_xy / self.global_path。
        如果你希望换文件，可以先通过别的节点修改参数 path_csv，再调用 /restart_path。
        """
        if not hasattr(self, 'path_csv') or not self.path_csv:
            raise RuntimeError("path_csv not set, cannot reload")

        llh = self.load_csv(self.path_csv)
        if len(llh) < 1:
            raise RuntimeError(f"No valid waypoints in CSV: {self.path_csv}")
        llh.sort(key=lambda x: x[0])

        # 重新设置 ENU 原点（用新 CSV 第一个点）
        lat0, lon0 = llh[0][1], llh[0][2]
        self.geo = LLA2ENU(lat0, lon0, 0.0)

        # 重建 ENU 路点
        self.waypoints_xy.clear()
        for _, lat, lon, hdg_deg_csv, pt_type in llh:
            x, y, _ = self.geo.lla_to_enu(lat, lon, 0.0)
            yaw_enu = heading_csv_deg_to_enu_rad(hdg_deg_csv)
            self.waypoints_xy.append((x, y, yaw_enu, int(pt_type)))

        # 重建 global_path（供 RViz 显示）
        self.global_path.poses.clear()
        for (x, y, _, _) in self.waypoints_xy:
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            self.global_path.poses.append(ps)

        self.get_logger().info(
            f"Reloaded path from {self.path_csv}, waypoints={len(self.waypoints_xy)}, "
            f"origin=({lat0:.7f},{lon0:.7f})"
        )

    # ---- helpers ----
    def _pub_global_path(self):
        t = self.get_clock().now().to_msg()
        self.global_path.header.stamp = t
        self.pub_global_path.publish(self.global_path)

    def load_csv(self, path) -> List[Tuple[int, float, float, float, int]]:
        wps = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0].startswith('#'):
                    continue
                try:
                    idx = int(row[0]); lat = float(row[1]); lon = float(row[2])
                    hdg = float(row[3]); pt = int(row[4]) if len(row) > 4 else 0
                    wps.append((idx, lat, lon, hdg, pt))
                except Exception:
                    continue
        return wps

    # ---- Inputs ----
    def on_drive_cmd(self, msg: UInt8):
        v = int(msg.data)
        if v == 1:
            self.drive_state = self.DS_RUNNING
            self.get_logger().info("Drive: RUNNING")
        elif v == 2:
            self.drive_state = self.DS_ESTOP
            self.get_logger().warn("Drive: ESTOP")
        else:
            self.drive_state = self.DS_PAUSED
            self.get_logger().info("Drive: PAUSED")

    def on_abort(self, msg: Bool):
        if bool(msg.data):
            self.drive_state = self.DS_ESTOP
            self.get_logger().warn("Drive: ESTOP (via /abort True)")

    def on_task_done(self, msg: Bool):
        if bool(msg.data):
            if self.waiting_for_task:
                self.waiting_for_task = False
                if self.seg_idx < len(self.waypoints_xy) - 2:
                    self.seg_idx += 1
                self.drive_state = self.DS_RUNNING
                self.task_done_latch = False
                self._pub_waiting(False)
                self.get_logger().info("Task done: resume following")
            else:
                self.task_done_latch = True

    def _pub_waiting(self, v: bool):
        self.pub_task_wait.publish(Bool(data=bool(v)))
        if v:
            self.get_logger().info("Enter task waiting state (/at_task_waiting = True)")
        else:
            self.get_logger().info("Exit task waiting state (/at_task_waiting = False)")

    # ---- Sensors ----
    def on_gps(self, msg: NavSatFix):
        if msg.status.status < 0:
            return
        alt = msg.altitude if msg.altitude == msg.altitude else 0.0
        x, y, _ = self.geo.lla_to_enu(msg.latitude, msg.longitude, alt)
        self.cur_x = x
        self.cur_y = y
        if self.dead_reckon:
            if self.dr_x is None:
                self.dr_x, self.dr_y = x, y
            else:
                a = max(0.0, min(1.0, self.dr_alpha))
                self.dr_x = a * self.dr_x + (1.0 - a) * x
                self.dr_y = a * self.dr_y + (1.0 - a) * y

    def on_imu(self, msg: Imu):
        if self.use_rtk_heading:
            return
        q = msg.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.cur_yaw = wrap_pi(yaw + self.yaw_offset)

    def on_rtk_heading(self, msg: Float32):
        hdg = float(msg.data)
        if not math.isfinite(hdg):
            return
        yaw_enu = heading_csv_deg_to_enu_rad(hdg)
        self.cur_yaw = wrap_pi(yaw_enu + self.yaw_offset)

    # ---- 主控制循环：计算 /stack_cmd/traj ----
    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-3
        now_sec = now.nanoseconds * 1e-9
        self.last_time = now

        if self.cur_x is None or self.cur_y is None or self.cur_yaw is None:
            return

        # 如果处于卸货模式，优先处理卸货计时
        if self.unload_mode:
            self.drive_state = self.DS_PAUSED
            self.waiting_for_task = False
            self._pub_waiting(False)

            # 阶段1：持续保持 unload=True
            if now_sec < self.unload_end_time:
                self.publish_traj_cmd(
                    self.vcu_speed_stop_kmh, 0.0, 0.0,
                    unload=True
                )

            # 阶段2：持续一小段时间发送 unload=False，确保VCU收到复位
            elif now_sec < self.unload_reset_end_time:
                self.publish_traj_cmd(
                    self.vcu_speed_stop_kmh, 0.0, 0.0,
                    unload=False
                )

            # 完成：退出卸货模式，并锁存“已卸货完成”
            else:
                self.unload_mode = False
                self.unload_finished_once = True
                self.publish_traj_cmd(
                    self.vcu_speed_stop_kmh, 0.0, 0.0,
                    unload=False
                )
                self.get_logger().info("Unload finished, unload reset done, latch unload_finished_once=True")

            # 这里不要用尚未定义的 px/py，直接用当前位置
            self.publish_map_to_odom(self.cur_x, self.cur_y, self.cur_yaw)
            return

        # DR integrate
        if self.dead_reckon:
            if self.dr_x is None:
                self.dr_x, self.dr_y = self.cur_x, self.cur_y
            else:
                v = 0.0
                if self.drive_state == self.DS_RUNNING:
                    if self.dr_use_gps_speed and (self.last_gps_speed is not None):
                        v = self.last_gps_speed
                self.dr_x += v * math.cos(self.cur_yaw) * dt
                self.dr_y += v * math.sin(self.cur_yaw) * dt

        px = self.dr_x if (self.dead_reckon and self.dr_x is not None) else self.cur_x
        py = self.dr_y if (self.dead_reckon and self.dr_y is not None) else self.cur_y

        # viz path
        t = self.get_clock().now().to_msg()
        ps = PoseStamped()
        ps.header.stamp = t
        ps.header.frame_id = self.map_frame
        ps.pose.position.x = float(px)
        ps.pose.position.y = float(py)
        ps.pose.orientation = yaw_to_quat(self.cur_yaw)
        self.traj_path.poses.append(ps)
        if len(self.traj_path.poses) > self.max_traj_len:
            self.traj_path.poses.pop(0)
        self.traj_path.header.stamp = t
        self.pub_traj_path.publish(self.traj_path)
        self.global_path.header.stamp = t
        self.pub_global_path.publish(self.global_path)

        n = len(self.waypoints_xy)
        if n == 0:
            return

        # latch task_done
        if self.waiting_for_task and self.task_done_latch:
            self.task_done_latch = False
            self.waiting_for_task = False
            if self.seg_idx < n - 2:
                self.seg_idx += 1
            self.drive_state = self.DS_RUNNING
            self._pub_waiting(False)
            self.get_logger().info("Task done latch: resume following")

        # 下一个路点的简单距离（用于 waiting/非运行态）
        next_idx = min(self.seg_idx + 1, n - 1) if n >= 2 else 0
        dist_to_next_simple = math.hypot(px - self.waypoints_xy[next_idx][0],
                                         py - self.waypoints_xy[next_idx][1])

        # 等待任务阶段：不再输出运动命令（速度/方向都为0）
        if self.waiting_for_task:
            self.spin_state = 'IDLE'
            self.angle_sign = 0
            self.publish_traj_cmd(self.vcu_speed_stop_kmh, 0.0, dist_to_next_simple)
            self.publish_map_to_odom(px, py, self.cur_yaw)
            return

        # 非 RUNNING：输出1个停止命令即可
        if self.drive_state != self.DS_RUNNING:
            self.spin_state = 'IDLE'
            self.angle_sign = 0
            self.publish_traj_cmd(self.vcu_speed_stop_kmh, 0.0, dist_to_next_simple)
            self.publish_map_to_odom(px, py, self.cur_yaw)
            return

        # 只有一个路点
        if n == 1:
            xj, yj, yaw_wp, pt_type = self.waypoints_xy[0]
            self.control_single_point(px, py, xj, yj, yaw_wp, pt_type, now_sec)
            self.publish_map_to_odom(px, py, self.cur_yaw)
            return

        # 自动对齐初始 yaw 到第一段方向（可选）
        if self.auto_align_yaw and not self.aligned and n >= 2:
            seg_hdg0 = math.atan2(self.waypoints_xy[1][1] - self.waypoints_xy[0][1],
                                  self.waypoints_xy[1][0] - self.waypoints_xy[0][0])
            self.yaw_offset += wrap_pi(seg_hdg0 - self.cur_yaw)
            self.aligned = True

        self.seg_idx = max(0, min(self.seg_idx, n - 2))
        i = self.seg_idx
        j = i + 1
        xi, yi, _, _ = self.waypoints_xy[i]
        xj, yj, yaw_wp, pt_type_j = self.waypoints_xy[j]

        sx = xj - xi
        sy = yj - yi
        seg_len = math.hypot(sx, sy)
        if seg_len < 1e-6:
            if j < n - 1:
                self.seg_idx += 1
            else:
                self.publish_traj_cmd(self.vcu_speed_stop_kmh, 0.0, 0.0)
            self.publish_map_to_odom(px, py, self.cur_yaw)
            return

        seg_len2 = seg_len * seg_len
        rx = px - xi
        ry = py - yi
        tproj = max(0.0, min(1.0, (rx * sx + ry * sy) / seg_len2))
        proj_x = xi + tproj * sx
        proj_y = yi + tproj * sy
        cross = sx * ry - sy * rx
        cte = math.copysign(math.hypot(px - proj_x, py - proj_y), cross)
        dist_to_next = math.hypot(px - xj, py - yj)

        # 期望航向
        use_wp_heading = (
            self.use_csv_heading and
            dist_to_next < self.heading_align_dist and
            (pt_type_j == 1 or j == n - 1)
        )
        if use_wp_heading:
            hdg_des = yaw_wp
        else:
            la_t = min(1.0, tproj + (self.lookahead_dist / max(0.01, seg_len)))
            la_x = xi + la_t * sx
            la_y = yi + la_t * sy
            hdg_des = math.atan2(la_y - py, la_x - px)
        heading_error = wrap_pi(hdg_des - self.cur_yaw)
        heading_err_deg = math.degrees(heading_error)

        has_next_seg = (j < n - 1)
        psi1 = math.atan2(sy, sx)
        if has_next_seg:
            sx2 = self.waypoints_xy[j + 1][0] - xj
            sy2 = self.waypoints_xy[j + 1][1] - yj
            psi2 = math.atan2(sy2, sx2)
            dpsi = wrap_pi(psi2 - psi1)
            hdg_err_to_next_deg = math.degrees(wrap_pi(psi2 - self.cur_yaw))
        else:
            psi2 = psi1
            dpsi = 0.0
            hdg_err_to_next_deg = heading_err_deg
        corner_angle_deg = math.degrees(dpsi)

        # 任务点：到点并对正 -> 进入 waiting 状态
        if pt_type_j == 1:
            hdg_err_to_wp = wrap_pi(yaw_wp - self.cur_yaw)
            hdg_err_to_wp_deg = math.degrees(hdg_err_to_wp)
            
            is_last_point = (j == n - 1)  # 最后一个点

            if dist_to_next <= self.wp_reached_dist:
                if abs(hdg_err_to_wp_deg) > self.stop_turn_tol_deg:
                    # 用 SPIN 逻辑对正
                    pre_kmh, angle_send_deg = self.decide_vcu_action(
                        heading_err_deg=hdg_err_to_wp_deg,
                        hdg_err_to_next_deg=hdg_err_to_wp_deg,
                        cte=cte,
                        dist_to_next=dist_to_next,
                        corner_angle_deg=0.0,
                        has_next_seg=False,
                        now_sec=now_sec
                    )
                    angle_send_deg = max(-self.vcu_angle_move_limit_deg,
                                         min(self.vcu_angle_move_limit_deg, angle_send_deg))
                    self.publish_traj_cmd(pre_kmh, angle_send_deg, dist_to_next)
                    self.publish_map_to_odom(px, py, self.cur_yaw)
                    return
                else:
                    # 已对正
                    if is_last_point:
                        # 已经完成过最后点卸货：只保持停车，不再重复进入卸货模式
                        if self.unload_finished_once:
                            self.drive_state = self.DS_PAUSED
                            self.waiting_for_task = False
                            self.spin_state = 'IDLE'
                            self.angle_sign = 0
                            self._pub_waiting(False)
                            self.publish_traj_cmd(self.vcu_speed_stop_kmh, 0.0, dist_to_next, unload=False)
                            self.publish_map_to_odom(px, py, self.cur_yaw)
                            return

                        # ---- 第一次到最后一个任务点：进入卸货模式 ----
                        self.unload_mode = True
                        self.unload_end_time = now_sec + 30.0            # 卸货保持 50 秒
                        self.unload_reset_end_time = self.unload_end_time + 2.0  # 再给 2 秒显式复位
                        self.drive_state = self.DS_PAUSED
                        self.waiting_for_task = False
                        self.spin_state = 'IDLE'
                        self.angle_sign = 0

                        # 不进入 waiting_for_task，防止草捆对准块接管
                        self._pub_waiting(False)

                        self.publish_traj_cmd(
                            self.vcu_speed_stop_kmh, 0.0, dist_to_next,
                            unload=True
                        )
                        self.get_logger().info("Enter unload mode at last waypoint, unload=1 for 50s then unload=0 for 2s")
                        self.publish_map_to_odom(px, py, self.cur_yaw)
                        return
                    else:
                        self.waiting_for_task = True
                        self.drive_state = self.DS_PAUSED
                        self.spin_state = 'IDLE'
                        self.angle_sign = 0
                        self._pub_waiting(True)
                        self.publish_traj_cmd(self.vcu_speed_stop_kmh, 0.0, dist_to_next)
                        self.publish_map_to_odom(px, py, self.cur_yaw)
                        return
            # 未到点：继续正常控制

        # 普通点：段切换
        if pt_type_j != 1:
            if (self.seg_idx < n - 2) and (tproj > self.t_advance_min and dist_to_next < self.advance_when_close):
                self.seg_idx += 1
                self.publish_map_to_odom(px, py, self.cur_yaw)
                return

        # 最后一段末端减速+停车
        is_last_segment = (self.seg_idx == n - 2)
        if is_last_segment:
            if dist_to_next < 0.5:
                self.spin_state = 'IDLE'
                self.angle_sign = 0
                self.publish_traj_cmd(self.vcu_speed_stop_kmh, 0.0, dist_to_next)
                self.publish_map_to_odom(px, py, self.cur_yaw)
                return
            if dist_to_next < max(0.8, 0.5 * self.advance_when_close):
                self.spin_state = 'IDLE'
                self.angle_sign = 0
                self.publish_traj_cmd(self.vcu_speed_slow_kmh, 0.0, dist_to_next)
                self.publish_map_to_odom(px, py, self.cur_yaw)
                return

        # ---- 核心离散动作决策 ----
        pre_kmh, angle_send_deg = self.decide_vcu_action(
            heading_err_deg=heading_err_deg,
            hdg_err_to_next_deg=hdg_err_to_next_deg,
            cte=cte,
            dist_to_next=dist_to_next,
            corner_angle_deg=corner_angle_deg,
            has_next_seg=has_next_seg,
            now_sec=now_sec
        )
        angle_send_deg = max(-self.vcu_angle_move_limit_deg,
                             min(self.vcu_angle_move_limit_deg, angle_send_deg))
        self.publish_traj_cmd(pre_kmh, angle_send_deg, dist_to_next)
        self.publish_map_to_odom(px, py, self.cur_yaw)

    # ---- 离散动作决策 + SPIN 状态机 ----
    def decide_vcu_action(self,
                           heading_err_deg: float,
                           hdg_err_to_next_deg: float,
                           cte: float,
                           dist_to_next: float,
                           corner_angle_deg: float,
                           has_next_seg: bool,
                           now_sec: float):
        abs_hdg = abs(heading_err_deg)
        abs_hdg_next = abs(hdg_err_to_next_deg)

        # 0. SPIN 状态机
        if self.spin_state == 'PRE_STOP':
            if now_sec < self.spin_pre_stop_until:
                return self.vcu_speed_stop_kmh, 0.0
            else:
                self.spin_state = 'SPIN'

        if self.spin_state == 'SPIN':
            cur_err = abs_hdg_next if self.spin_use_next_seg else abs_hdg
            if cur_err <= self.vcu_angle_spin_exit_deg:
                self.spin_state = 'IDLE'
                self.angle_sign = 0
            else:
                angle_deg = self.spin_sign * self.vcu_angle_spin_cmd_deg
                return self.vcu_speed_stop_kmh, angle_deg

        # 1. 只有在 IDLE 才考虑启动新 SPIN
        is_corner_area = False
        if self.strict_corner_mode and has_next_seg:
            if abs(math.radians(corner_angle_deg)) >= self.corner_sharp_rad_strict \
               and dist_to_next <= self.corner_start_dist:
                is_corner_area = True

        want_spin_corner = is_corner_area and (abs_hdg_next >= self.vcu_angle_spin_enter_deg)
        want_spin_emergency = abs_hdg >= self.emergency_spin_hdg_deg

        if self.spin_state == 'IDLE' and (want_spin_corner or want_spin_emergency):
            already_stop = abs(self.last_pre_kmh_sent) < 0.1
            if want_spin_corner:
                self.spin_use_next_seg = True
                self.spin_sign = -1.0 if hdg_err_to_next_deg > 0.0 else 1.0
            else:
                self.spin_use_next_seg = False
                self.spin_sign = -1.0 if heading_err_deg > 0.0 else 1.0

            if already_stop:
                self.spin_state = 'SPIN'
                angle_deg = self.spin_sign * self.vcu_angle_spin_cmd_deg
                return self.vcu_speed_stop_kmh, angle_deg
            else:
                self.spin_state = 'PRE_STOP'
                self.spin_pre_stop_until = now_sec + self.spin_pre_stop_time_s
                return self.vcu_speed_stop_kmh, 0.0

        # 2. MOVE 模式：直行/小弯/大弯
        if abs_hdg <= self.hdg_deadband_deg and abs(cte) <= self.cte_deadband_m:
            self.angle_sign = 0
            pre_kmh = self.vcu_speed_fast_kmh
            self.last_pre_kmh_sent = pre_kmh
            return pre_kmh, 0.0

        sign = self.update_angle_sign(heading_err_deg)

        if abs_hdg < self.vcu_turn_small_thresh_deg:
            angle_deg = sign * self.vcu_turn_small_cmd_deg
            pre_kmh = self.vcu_speed_fast_kmh
        elif abs_hdg < self.vcu_turn_large_thresh_deg:
            angle_deg = sign * self.vcu_turn_small_cmd_deg
            pre_kmh = self.vcu_speed_slow_kmh
        else:
            angle_deg = sign * self.vcu_turn_large_cmd_deg
            pre_kmh = self.vcu_speed_slow_kmh

        if abs(cte) > 0.5:
            pre_kmh = min(pre_kmh, self.vcu_speed_slow_kmh)

        self.last_pre_kmh_sent = pre_kmh
        return pre_kmh, angle_deg

    def update_angle_sign(self, heading_err_deg: float) -> int:
        if self.angle_sign >= 0 and heading_err_deg > self.sign_hysteresis_deg:
            self.angle_sign = -1
        elif self.angle_sign <= 0 and heading_err_deg < -self.sign_hysteresis_deg:
            self.angle_sign = +1
        if self.angle_sign == 0:
            self.angle_sign = -1 if heading_err_deg >= 0.0 else +1
        return self.angle_sign

    def control_single_point(self, px, py, xj, yj, yaw_wp, pt_type, now_sec: float):
        dist = math.hypot(px - xj, py - yj)
        seg_hdg = math.atan2(yj - py, xj - px)
        use_wp_heading = (
            self.use_csv_heading and
            dist < self.heading_align_dist and
            (pt_type == 1)
        )
        hdg_des = yaw_wp if use_wp_heading else seg_hdg
        heading_error = wrap_pi(hdg_des - self.cur_yaw)
        heading_err_deg = math.degrees(heading_error)

        if pt_type == 1 and dist <= self.wp_reached_dist:
            hdg_err_to_wp = wrap_pi(yaw_wp - self.cur_yaw)
            hdg_err_to_wp_deg = math.degrees(hdg_err_to_wp)
            if abs(hdg_err_to_wp_deg) > self.stop_turn_tol_deg:
                pre_kmh, angle_deg = self.decide_vcu_action(
                    heading_err_deg=hdg_err_to_wp_deg,
                    hdg_err_to_next_deg=hdg_err_to_wp_deg,
                    cte=0.0,
                    dist_to_next=dist,
                    corner_angle_deg=0.0,
                    has_next_seg=False,
                    now_sec=now_sec
                )
                angle_deg = max(-self.vcu_angle_move_limit_deg,
                                min(self.vcu_angle_move_limit_deg, angle_deg))
                self.publish_traj_cmd(pre_kmh, angle_deg, dist)
            else:
                self.waiting_for_task = True
                self.drive_state = self.DS_PAUSED
                self.spin_state = 'IDLE'
                self.angle_sign = 0
                self._pub_waiting(True)
                self.publish_traj_cmd(self.vcu_speed_stop_kmh, 0.0, dist)
            return

        pre_kmh, angle_send_deg = self.decide_vcu_action(
            heading_err_deg=heading_err_deg,
            hdg_err_to_next_deg=heading_err_deg,
            cte=0.0,
            dist_to_next=dist,
            corner_angle_deg=0.0,
            has_next_seg=False,
            now_sec=now_sec
        )
        angle_send_deg = max(-self.vcu_angle_move_limit_deg,
                             min(self.vcu_angle_move_limit_deg, angle_send_deg))
        self.publish_traj_cmd(pre_kmh, angle_send_deg, dist)

    def publish_traj_cmd(self, 
                         pre_kmh: float, 
                         angle_deg: float, 
                         dist_m: float,
                         pick: bool = False,
                         unload: bool = False,
                         dump: bool = False,
                         pick_action: bool = False):
        cmd = StackCommand()
        cmd.pre_speed_kmh = float(pre_kmh)
        cmd.angle_deg = float(angle_deg)
        cmd.dist_to_target_m = float(dist_m)
        cmd.pick = bool(pick)
        cmd.unload = bool(unload)
        cmd.dump = bool(dump)
        cmd.pick_action = bool(pick_action)
        cmd.valid = True
        self.pub_traj_cmd.publish(cmd)

    def publish_map_to_odom(self, px: float, py: float, yaw_map_base: float):
        try:
            tf_ob = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
            tx_ob = tf_ob.transform.translation.x
            ty_ob = tf_ob.transform.translation.y
            rz = tf_ob.transform.rotation
            yaw_ob = math.atan2(
                2.0 * (rz.w * rz.z + rz.x * rz.y),
                1.0 - 2.0 * (rz.y * rz.y + rz.z * rz.z)
            )

            yaw_mo = wrap_pi(yaw_map_base - yaw_ob)
            cos_mo = math.cos(yaw_mo)
            sin_mo = math.sin(yaw_mo)
            tmo_x = px - (cos_mo * tx_ob - sin_mo * ty_ob)
            tmo_y = py - (sin_mo * tx_ob + cos_mo * ty_ob)

            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = self.map_frame
            tf.child_frame_id = self.odom_frame
            tf.transform.translation.x = float(tmo_x)
            tf.transform.translation.y = float(tmo_y)
            tf.transform.translation.z = 0.0
            tf.transform.rotation = yaw_to_quat(yaw_mo)
            self.tfb.sendTransform(tf)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TrajWaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
