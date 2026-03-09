#!/usr/bin/env python3
import math
import csv
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, UInt8


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class LLA2ENU:
    """
    与你现有栈一致的 LLA->ENU 转换类。
    """

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
        # ECEF->ENU 旋转矩阵
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


def point_in_polygon(px: float, py: float, poly: List[Tuple[float, float]]) -> bool:
    """
    射线法判断点是否在多边形内部（含边界视为 inside）。
    poly: [(x0,y0), (x1,y1), ...] 顶点按顺/逆时针给出。
    """
    inside = False
    n = len(poly)
    if n < 3:
        return False
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        # 判断 y 在边的两个端点 y 之间（半开区间避免重复）
        if ((y1 > py) != (y2 > py)):
            # 计算交点 x 坐标
            x_inter = (x2 - x1) * (py - y1) / (y2 - y1 + 1e-9) + x1
            if px < x_inter:
                inside = not inside
    return inside


def distance_point_to_segment(px: float, py: float,
                              x1: float, y1: float,
                              x2: float, y2: float) -> float:
    """
    点到线段的最短距离（欧氏距离）
    """
    vx = x2 - x1
    vy = y2 - y1
    wx = px - x1
    wy = py - y1
    seg_len2 = vx * vx + vy * vy
    if seg_len2 < 1e-12:
        return math.hypot(px - x1, py - y1)
    t = (wx * vx + wy * vy) / seg_len2
    if t < 0.0:
        tx, ty = x1, y1
    elif t > 1.0:
        tx, ty = x2, y2
    else:
        tx = x1 + t * vx
        ty = y1 + t * vy
    return math.hypot(px - tx, py - ty)


class GeofenceMonitor(Node):
    """
    电子围栏监控：
      - 从 fence_csv 加载多边形围栏点（经纬度），转成 ENU 多边形；
      - 订阅 GPS (NavSatFix)，实时将车位置转为 ENU 点；
      - 判断是否在多边形内 + 距离边界的最小距离；
      - 发布:
          /geofence_state (UInt8): 0=inside, 1=warn, 2=outside/unknown
          /geofence_ok    (Bool):  True=inside且距离>warn_margin, False=warn或outside
      - 当 inside/warn -> outside 转变时，触发一次 /abort True (ESTOP)。
    """

    STATE_INSIDE = 0
    STATE_WARN   = 1
    STATE_OUT    = 2   # 包含越界或未知情况

    def __init__(self):
        super().__init__('geofence_monitor')

        # 参数
        self.declare_parameter('fence_csv', '')
        self.declare_parameter('gps_topic', '/fix_center')
        self.declare_parameter('warn_margin_m', 5.0)       # 距离边界 < warn_margin -> WARN
        self.declare_parameter('check_period_s', 0.2)      # 检查周期
        self.declare_parameter('abort_on_outside', True)   # 越界是否触发 /abort

        fence_csv = self.get_parameter('fence_csv').get_parameter_value().string_value
        if not fence_csv:
            self.get_logger().error("fence_csv is required for geofence_monitor.")
            raise SystemExit

        self.gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        self.warn_margin_m = float(self.get_parameter('warn_margin_m').value)
        self.check_period_s = float(self.get_parameter('check_period_s').value)
        self.abort_on_outside = bool(self.get_parameter('abort_on_outside').value)

        # 读取围栏点并构建 ENU 多边形
        ll = self.load_fence_csv(fence_csv)
        if len(ll) < 3:
            self.get_logger().error(f"Need at least 3 points in fence_csv: {fence_csv}")
            raise SystemExit

        # 使用第一个点作为 ENU 原点
        lat0, lon0 = ll[0]
        self.geo = LLA2ENU(lat0, lon0, 0.0)

        self.fence_xy: List[Tuple[float, float]] = []
        for (lat, lon) in ll:
            x, y, _ = self.geo.lla_to_enu(lat, lon, 0.0)
            self.fence_xy.append((x, y))

        self.get_logger().info(
            f"Geofence loaded from {fence_csv}, vertices={len(self.fence_xy)}, "
            f"origin=({lat0:.7f},{lon0:.7f})"
        )

        # 当前位置（ENU）
        self.px_enu: Optional[float] = None
        self.py_enu: Optional[float] = None

        # 订阅 GPS
        self.create_subscription(NavSatFix, self.gps_topic, self.on_gps, 10)

        # latched QoS 用于状态话题
        qos_latched = QoSProfile(depth=1)
        qos_latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # 发布状态
        self.pub_state = self.create_publisher(UInt8, '/geofence_state', qos_latched)
        self.pub_ok    = self.create_publisher(Bool,  '/geofence_ok',    qos_latched)

        # 触发 ESTOP 的 /abort
        self.pub_abort = self.create_publisher(Bool, '/abort', 1)

        # 上一次状态
        self.last_state: Optional[int] = None

        # 定时器
        self.create_timer(self.check_period_s, self.check_loop)

        self.get_logger().info(
            f"GeofenceMonitor started. GPS:{self.gps_topic}, warn_margin={self.warn_margin_m} m"
        )

    # ---------- CSV 读取 ----------
    def load_fence_csv(self, path: str) -> List[Tuple[float, float]]:
        """
        CSV 格式: idx, lat, lon
        忽略注释行(#)与空行，按 idx 排序。
        """
        pts: List[Tuple[int, float, float]] = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0].startswith('#'):
                    continue
                try:
                    idx = int(row[0])
                    lat = float(row[1])
                    lon = float(row[2])
                    pts.append((idx, lat, lon))
                except Exception:
                    continue
        pts.sort(key=lambda x: x[0])
        return [(lat, lon) for (_, lat, lon) in pts]

    # ---------- GPS 回调 ----------
    def on_gps(self, msg: NavSatFix):
        if msg.status.status < 0:
            # 无效 GPS
            return
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude if msg.altitude == msg.altitude else 0.0
        x, y, _ = self.geo.lla_to_enu(lat, lon, alt)
        self.px_enu = x
        self.py_enu = y

    # ---------- 检查逻辑 ----------
    def check_loop(self):
        # 没有位置就视为 unknown/outside
        if self.px_enu is None or self.py_enu is None:
            self.set_state(self.STATE_OUT, ok=False, reason="no GPS position yet")
            return

        px = float(self.px_enu)
        py = float(self.py_enu)

        inside = point_in_polygon(px, py, self.fence_xy)
        if not inside:
            # 越界
            self.set_state(self.STATE_OUT, ok=False, reason="outside polygon")
            return

        # 在多边形内，计算到边界的最近距离
        dmin = self.compute_min_distance_to_edges(px, py, self.fence_xy)

        if dmin < self.warn_margin_m:
            # 临近边界
            self.set_state(self.STATE_WARN, ok=False, reason=f"near boundary, dmin={dmin:.2f}m")
        else:
            # 安全区
            self.set_state(self.STATE_INSIDE, ok=True, reason=f"inside, dmin={dmin:.2f}m")

    def compute_min_distance_to_edges(self,
                                      px: float,
                                      py: float,
                                      poly: List[Tuple[float, float]]) -> float:
        n = len(poly)
        if n < 2:
            return float('inf')
        dmin = float('inf')
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            d = distance_point_to_segment(px, py, x1, y1, x2, y2)
            if d < dmin:
                dmin = d
        return dmin

    def set_state(self, state: int, ok: bool, reason: str = ""):
        # 发布状态（latched）
        self.pub_state.publish(UInt8(data=int(state)))
        self.pub_ok.publish(Bool(data=bool(ok)))

        # 状态变化时打 log 并在从 inside/warn -> outside 时触发 /abort
        if self.last_state is None or self.last_state != state:
            self.last_state = state
            if state == self.STATE_INSIDE:
                self.get_logger().info(f"Geofence: INSIDE. {reason}")
            elif state == self.STATE_WARN:
                self.get_logger().warn(f"Geofence: WARN (near boundary). {reason}")
            elif state == self.STATE_OUT:
                self.get_logger().error(f"Geofence: OUTSIDE! {reason}")
                if self.abort_on_outside:
                    self.trigger_abort_once()

    def trigger_abort_once(self):
        """
        越界时发一次 /abort True。
        可根据需要扩展成带复位逻辑（目前不自动清除）。
        """
        self.pub_abort.publish(Bool(data=True))
        self.get_logger().warn("Geofence: publish /abort = True (ESTOP)")
        # 如果你希望自动清除，可在恢复 inside 后再发 False，这里先不自动解锁。


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
