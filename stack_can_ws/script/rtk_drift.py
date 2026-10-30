#!/usr/bin/env python3
"""Manual origin capture and drift update with five-second stationary sampling."""
import copy
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time


def estimate(samples):
    if len(samples) < 10 or samples[-1][0] - samples[0][0] < 4.5:
        raise ValueError('有效采样不足5秒或少于10个点')
    lat = statistics.median(p[1] for p in samples)
    lon = statistics.median(p[2] for p in samples)
    scale = 111320.0 * math.cos(math.radians(lat))
    kept = [p for p in samples if math.hypot((p[1]-lat)*111320, (p[2]-lon)*scale) <= 0.5]
    if len(kept) < max(10, len(samples)*0.8):
        raise ValueError('定位离散过大，请保持静止并检查RTK')
    return [statistics.mean(p[i] for p in kept) for i in (1, 2)]


def validate(point):
    if len(point) != 2 or not all(math.isfinite(x) for x in point) or not -90 < point[0] < 90 or not -180 <= point[1] <= 180:
        raise ValueError('基准坐标无效')
    return point


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import Bool, String
    rclpy.init()
    node = Node('rtk_drift_wait' if '--wait' in sys.argv else 'rtk_drift')
    ready_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    ready = [False]
    if '--wait' in sys.argv:
        sub = node.create_subscription(Bool, '/rtk_drift/ready', lambda m: ready.__setitem__(0, m.data), ready_qos)
        deadline = time.monotonic()+40
        while rclpy.ok() and not ready[0] and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()
        return 0 if ready[0] else 3
    path = Path(os.environ.get('RTK_DRIFT_BASELINE', '/root/stack_can_ws/data/rtk_drift_baseline.json'))
    baseline = None
    if path.exists():
        data = json.loads(path.read_text())
        if data['version'] != 1:
            raise ValueError('不支持的基准文件版本')
        baseline = validate(data['position'])
    pub = node.create_publisher(NavSatFix, '/fix_center', 10)
    state = node.create_publisher(Bool, '/rtk_drift/ready', ready_qos)
    state.publish(Bool(data=False))
    samples = []
    correction_path = path.with_name('rtk_drift_correction.json')
    offset = [0.0, 0.0]
    if correction_path.exists():
        saved = json.loads(correction_path.read_text())
        if saved['baseline'] == baseline:
            offset = saved['offset']
        else:
            node.get_logger().warn('原点已更新，忽略旧原点的补偿文件')
        if len(offset) != 2 or not all(math.isfinite(x) for x in offset):
            raise ValueError('补偿量无效')
    pending = None
    action = None
    result_pub = node.create_publisher(String, '/rtk_drift/result', 10)

    def respond(ok, message):
        nonlocal pending
        result_pub.publish(String(data=json.dumps(dict(type='rtk_drift_result', request_id=pending,
            ok=ok, message=message), ensure_ascii=False)))
        pending = None

    def request(msg):
        nonlocal pending, started, action
        data = json.loads(msg.data)
        if pending is not None:
            return
        pending = data['request_id']
        action = data.get('action')
        if action not in ('origin', 'drift'):
            respond(False, '必须指定origin或drift操作')
            return
        if action == 'drift' and baseline is None:
            respond(False, '尚未标定原点，请先按标定原点键')
            return
        samples.clear()
        started = time.monotonic()
        node.get_logger().info('手动RTK采样开始，请保持在参考点静止5秒')

    request_sub = node.create_subscription(String, '/rtk_drift/calibrate', request, 10)
    last = None
    started = time.monotonic()

    def receive(msg):
        nonlocal offset, last, baseline
        now = time.monotonic()
        if msg.status.status < 0 or not all(math.isfinite(v) for v in (msg.latitude, msg.longitude)):
            return
        if not -90 < msg.latitude < 90 or not -180 <= msg.longitude <= 180:
            return
        if pending is not None:
            # Center node can republish the same fix on heading updates.
            stamp = (msg.latitude, msg.longitude, msg.header.stamp.sec, msg.header.stamp.nanosec)
            if stamp == last:
                return
            last = stamp
            if samples and now-samples[-1][0] > 1.0:
                samples.clear()
            samples.append((now, msg.latitude, msg.longitude))
            if now-samples[0][0] < 5.0:
                return
            try:
                current = estimate(samples)
                if action == 'origin':
                    candidate_baseline = current
                    candidate = [0.0, 0.0]
                    path.parent.mkdir(parents=True, exist_ok=True)
                    temporary_origin = path.with_suffix('.tmp')
                    with temporary_origin.open('w') as stream:
                        json.dump({'version': 1, 'position': current, 'created_at': time.time()}, stream)
                        stream.flush()
                        os.fsync(stream.fileno())
                    temporary_origin.replace(path)
                    baseline = candidate_baseline
                    offset = [0.0, 0.0]
                else:
                    candidate = [current[i]-baseline[i] for i in range(2)]
                temporary = correction_path.with_suffix('.tmp')
                with temporary.open('w') as stream:
                    json.dump(dict(baseline=baseline, offset=candidate), stream)
                    stream.flush()
                    os.fsync(stream.fileno())
                temporary.replace(correction_path)
                offset = candidate
                respond(True, ('原点已覆盖，旧补偿已清零' if action == 'origin' else '漂移量已更新，原点不变')+'；保持暂停，请手动开始循迹')
                node.get_logger().info('RTK标定完成，纬度/经度补偿量: '+str(offset))
                state.publish(Bool(data=True))
            except Exception as exc:
                node.get_logger().error(str(exc))
                respond(False, str(exc))
        result = copy.deepcopy(msg)
        result.latitude -= offset[0]
        result.longitude -= offset[1]
        pub.publish(result)

    sub = node.create_subscription(NavSatFix, '/fix_center_raw', receive, qos_profile_sensor_data)
    node.get_logger().info('手动标定模式：启动不采样，使用已保存补偿或零补偿')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if pending is not None and time.monotonic()-started > 30:
                respond(False, 'RTK标定超时，未更新补偿')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
