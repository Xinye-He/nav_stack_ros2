#!/usr/bin/env python3
"""Read-only ROS2 terminal dashboard backed by the authoritative 9011 snapshot."""
import asyncio
from datetime import datetime
import json
import queue
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from websockets.legacy.client import connect

LABELS = {'Neutral': 'Neutral', 'starting': '启动中', 'startup_failed': '启动失败', 'ready': '就绪',
          'runtime_fault': '运行故障', 'error': '错误', 'trajectory': '循迹中',
          'remote': '9010遥控模式', 'alarm': '报警', 'warning': '警告',
          'clear': '安全距离', 'unknown': '未知'}


def label(value):
    return LABELS.get(value, str(value) if value is not None else '—')


def event_time(event):
    stamp = event.get('ros_stamp') or {}
    if isinstance(stamp.get('sec'), (int, float)) and stamp['sec'] > 0:
        return stamp['sec'] + stamp.get('nanosec', 0) / 1e9
    try:
        return datetime.fromisoformat(event.get('received_at', '').replace('Z', '+00:00')).timestamp()
    except (ValueError, TypeError):
        return None


def fault_summary(snapshot):
    emergency = snapshot.get('emergency_status', {})
    if emergency.get('active'):
        return emergency['message']
    message = snapshot.get('device_message') or '无'
    for code, text in (('network_disconnected','9010遥控连接断开'),('network_heartbeat_timeout','9010遥控心跳超时'),('network_unconfirmed','网络故障处置未确认')):
        message = message.replace('车辆已急停/中止：' + code, text + '，车辆已急停，等待手动暂停复位')
    if snapshot.get('device_state') != 'runtime_fault' or 'abort' not in message.lower():
        return message
    events = snapshot.get('diagnostics', {}).get('events', [])
    # Device lifecycle retains its FIRST fault. Use the first observed ESTOP,
    # never a later unrelated network failure from the log history.
    estop = next((e for e in events if e.get('source') == 'stack_can_executor'
                  and 'safety=ESTOP' in e.get('message', '')), None)
    at = event_time(estop) if estop else None
    complete = not events or events[0].get('event_id', 1) == 1
    if not complete or at is None or 'estop_reason=abort' not in estop.get('message', ''):
        reason = '车辆急停：来源未记录（缺少对应触发日志）'
    else:
        matched = []
        for event in events:
            when = event_time(event)
            if event.get('evidence') == 'trigger_report' and when is not None and abs(when - at) <= 3.0:
                matched.append(event)
        names = {'websocket_teleop_bridge': '9010网络断联保护',
                 'motor_protection_node': '电机保护', 'geofence_monitor': '电子围栏保护'}
        parts = []
        for event in matched:
            text = '{} [{}]：{}'.format(names.get(event.get('source'), event.get('source', '未知节点')),
                                         event.get('source', ''), event.get('message', ''))
            if text not in parts:
                parts.append(text)
        reason = ('车辆急停，关联触发：' + '；'.join(parts)) if parts else '车辆急停：来源未记录（未找到同时间的触发日志）'
    # Keep other simultaneous failure messages instead of hiding them.
    other = [part for part in message.split('；') if 'abort' not in part.lower()]
    return '；'.join(other + [reason])


def render(snapshot, age, execution, execution_age, detail='', show_diagnostics=False):
    fresh = snapshot is not None and age <= 3.0
    state = snapshot.get('device_state_label', label(snapshot.get('device_state'))) if fresh else ('断线/状态超时' if snapshot else '等待连接')
    lines = ['Jetson 设备状态（只读，Ctrl+C退出）', '=' * 58,
             '设备：{}    状态接收距今：{}'.format(state, '{:.1f}s'.format(age) if snapshot else '—')]
    if not fresh:
        lines += ['车辆模式：—', '外设：状态不可用（不使用历史快照判断就绪）']
        if detail:
            lines.append('连接：' + detail)
        return '\n'.join(lines)
    lines += ['原因：' + fault_summary(snapshot),
              '车辆模式：' + label(snapshot.get('vehicle_state'))]
    network = snapshot.get('network_status', {})
    if network.get('severity') == 'warning':
        lines.append('网络警告：' + network.get('message', '上位机断连'))
    if execution and execution_age <= 3.0:
        lines += ['执行器：{}  控制模式：{}  drive_state：{}'.format(
            execution.get('safety_state', '—'), execution.get('control_mode', '—'), execution.get('drive_state', '—'))]
    else:
        lines.append('执行器实时状态：等待数据/超时')
    lines.append('-' * 58)
    for key, name in [('rtk', 'RTK'), ('lidar', '激光雷达'), ('ultrasonic', '超声波'), ('can', 'CAN')]:
        item = snapshot.get('peripherals', {}).get(key, {})
        lines.append('{}：{}    数据年龄：{} ms'.format(name, label(item.get('state')), item.get('data_age_ms', '—')))
        if key == 'can' and item.get('bus'):
            bus = item['bus']
            lines.append('  总线：{}  波特率：{}  错误计数 TX/RX：{}/{}  状态年龄：{} ms'.format(
                bus.get('state', '未知'), bus.get('bitrate', '—'), bus.get('tx_error_counter', '—'),
                bus.get('rx_error_counter', '—'), bus.get('status_age_ms', '—')))
        for error in item.get('errors', []):
            lines.append('  {}：{}'.format(error.get('code', ''), error.get('message', '')))
    obstacle = snapshot.get('obstacle_warning', {})
    lines += ['-' * 58, '障碍物预警：{}（仅显示预警）'.format(label(obstacle.get('state')))]
    for key, name in [('front_left', '左前'), ('front_right', '右前'), ('rear_left', '左后'), ('rear_right', '右后')]:
        distance = obstacle.get('distances_m', {}).get(key)
        lines.append('{}：{}  {}'.format(name, '{:.2f} m'.format(distance) if isinstance(distance, (int, float)) else '—',
                     label(obstacle.get('channel_states', {}).get(key))))
    if show_diagnostics:
        events = snapshot.get('diagnostics', {}).get('events', [])
        lines += ['-' * 58, '最近诊断记录（历史记录，不代表当前故障）']
        for event in events[-5:]:
            kind = '触发记录' if event.get('evidence') == 'trigger_report' else '相关日志'
            lines.append('{} [{}] {} {}：{}'.format(event.get('received_at', ''), kind, event.get('source', ''), event.get('level', ''), event.get('message', '')))
        if not events:
            lines.append('暂无记录；仅采集状态服务启动后的日志')
    lines += ['boot_id：' + str(snapshot.get('boot_id', '—')),
              'seq：{}    时间：{}'.format(snapshot.get('seq', '—'), snapshot.get('timestamp', '—'))]
    return '\n'.join(lines)


class DeviceStatusViewer(Node):
    def __init__(self):
        super().__init__('device_status_viewer')
        self.declare_parameter('status_url', 'ws://127.0.0.1:9011/status')
        self.declare_parameter('display', True)
        self.declare_parameter('show_diagnostics', False)
        self.show_diagnostics = self.get_parameter('show_diagnostics').value
        self.url = self.get_parameter('status_url').value
        self.show = self.get_parameter('display').value
        self.pub = self.create_publisher(String, '/device/status', 10)
        self.create_subscription(String, '/vehicle/execution_status', self.on_execution, 10)
        self.execution, self.execution_at = {}, 0.0
        self.snapshot, self.received = None, 0.0
        self.detail = ''
        self.inbox = queue.Queue(maxsize=100)
        self.stop = threading.Event()
        self.previous = None
        self.create_timer(0.2, self.tick)
        self.worker = threading.Thread(target=lambda: asyncio.run(self.receive()), daemon=True)
        self.worker.start()

    def on_execution(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self.execution, self.execution_at = value, time.monotonic()
        except (ValueError, TypeError):
            pass

    def enqueue(self, value):
        try:
            self.inbox.put_nowait(value)
        except queue.Full:
            try:
                self.inbox.get_nowait()
            except queue.Empty:
                pass
            self.inbox.put_nowait(value)

    async def receive(self):
        while not self.stop.is_set():
            try:
                async with connect(self.url, open_timeout=3, close_timeout=1) as ws:
                    while not self.stop.is_set():
                        try:
                            raw = await asyncio.wait_for(ws.recv(), timeout=1)
                        except asyncio.TimeoutError:
                            continue
                        try:
                            value = json.loads(raw)
                            if isinstance(value, dict) and value.get('type') == 'device_status':
                                self.enqueue((value, time.monotonic(), ''))
                        except (ValueError, TypeError):
                            continue
            except Exception as exc:
                self.enqueue((None, time.monotonic(), str(exc)))
                for _ in range(10):
                    if self.stop.is_set():
                        return
                    await asyncio.sleep(0.1)

    def tick(self):
        while True:
            try:
                value, received, detail = self.inbox.get_nowait()
            except queue.Empty:
                break
            self.detail = detail
            if value is not None:
                self.snapshot, self.received = value, received
                self.pub.publish(String(data=json.dumps(value, ensure_ascii=False, allow_nan=False)))
        if not self.show:
            return
        now = time.monotonic()
        text = render(self.snapshot, now - self.received, self.execution, now - self.execution_at, self.detail, self.show_diagnostics)
        # Redirected output prints meaningful changes without a continuous timestamp flood.
        signature = '\n'.join(line for line in text.splitlines() if not line.startswith(('seq：', '设备：')))
        signature += str(self.snapshot.get('device_state') if self.snapshot and now - self.received <= 3 else 'offline')
        if sys.stdout.isatty():
            print('\033[2J\033[H' + text, flush=True)
        elif signature != self.previous:
            print(text + '\n', flush=True)
        self.previous = signature


def main(args=None):
    rclpy.init(args=args)
    node = DeviceStatusViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop.set()
        node.worker.join(timeout=5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
