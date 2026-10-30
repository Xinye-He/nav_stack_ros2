#!/usr/bin/env python3
"""Read-only WebSocket status service. Survives demo launch termination."""
import argparse
import asyncio
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace

import rclpy
from rcl_interfaces.msg import Log
from std_msgs.msg import String, Bool
from websockets.legacy.server import serve

from obstacle_warning import summarize
from health_monitor import StartupHealthCheck
from status_model import vehicle_state, fault_presentation
from status_diagnostics import DiagnosticEvents
from status_store import read, update
from can_bus_status import enrich_can_status, poll_can_bus
from network_status import summarize_network


class StatusService(StartupHealthCheck):
    def __init__(self, startup_timeout, runtime_timeout, obstacle_threshold=0.5, obstacle_warning_threshold=2.0):
        args = SimpleNamespace(node_name='device_status_monitor', check_fix=True, check_heading=True,
            check_lidar=True, check_ultrasonic=True, check_can=True,
            fix_topic='/fix', heading_topic='/heading_deg', lidar_topic='/rslidar_points',
            ultrasonic_topic='/ultrasonic_distances', can_feedback_topic='/stack_can/feedback',
            startup_timeout=startup_timeout, runtime_timeout=runtime_timeout, once=False)
        self.can_bus_status = None
        self.can_bus_received = None
        self.can_bus_error = ''
        self.obstacle_ranges = None
        self.obstacle_received = None
        self.obstacle_threshold = obstacle_threshold
        self.obstacle_warning_threshold = obstacle_warning_threshold
        self.reset_guard_pub = None
        self.network_event = None
        self.pending_network_ack = None
        self.control = {}
        self.control_time = None
        self.link_ok = False
        self.link_time = None
        self.driver_errors = {}
        self.diagnostics = DiagnosticEvents()
        self.ready_nodes = set()
        self.seq = 0
        super().__init__(args)
        self.reset_guard_pub = self.create_publisher(Bool, '/safety/reset_peripherals_ready', 10)
        self.create_subscription(String, '/vehicle/execution_status', self.on_control, 10)
        self.create_subscription(Bool, '/network_link_ok', self.on_link, 10)
        self.create_subscription(String, '/safety/network_result', self.on_network_event, 10)
        self.create_subscription(String, '/vehicle/reset_result', self.on_reset_result, 10)
        self.create_subscription(Log, '/rosout', self.on_log, 100)
        self.snapshot = {}

    def ultrasonic_cb(self, msg):
        # Parent callback assesses data availability only. Distance never enters health faults.
        super().ultrasonic_cb(msg)
        self.obstacle_received = time.monotonic()
        self.obstacle_ranges = list(msg.data[:4]) if len(msg.data) >= 4 else None

    def on_control(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
            self.control = value
            self.control_time = time.monotonic()
        except (ValueError, TypeError):
            return

    def on_network_event(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self.network_event = value
        except (ValueError, TypeError):
            pass

    def on_reset_result(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict) and value.get('ok') and value.get('cleared_reason') in ('', 'drive_cmd','network_disconnected','network_heartbeat_timeout','network_unconfirmed'):
                self.pending_network_ack = (value, time.monotonic())
        except (ValueError, TypeError):
            pass

    def on_link(self, msg):
        self.link_ok = bool(msg.data)
        self.link_time = time.monotonic()

    def on_log(self, msg):
        self.diagnostics.record(msg)
        if msg.level < 40:
            return
        component = {'nmea_bridge': 'rtk', 'rsview': 'lidar',
                     'ultrasonic': 'ultrasonic', 'can_feedback_node': 'can'}.get(msg.name)
        if component:
            self.driver_errors[component] = {'code': 'DRIVER_ERROR',
                'message': msg.msg[:1200], 'source': msg.name}

    def check_once(self):
        now = time.monotonic()
        lifecycle = read()
        if not lifecycle:
            lifecycle = update('starting', '等待外设启动', phase='initializing')
        state = lifecycle['device_state']
        grace = now - self.start_time < self.args.startup_timeout
        peripherals = self.peripheral_status(self.args.runtime_timeout, grace)
        for component, item in peripherals.items():
            if item['state'] == 'ready':
                self.driver_errors.pop(component, None)
            elif component in self.driver_errors:
                item['state'] = 'error'
                item['errors'].append(self.driver_errors[component])
        if 'can' in peripherals:
            peripherals['can'] = enrich_can_status(peripherals['can'], self.can_bus_status,
                self.can_bus_received, now, grace, self.can_bus_error)
        required = {'stack_can_executor', 'traj_waypoint_follower', 'geofence_monitor',
                    'dr_odometry', 'motor_protection_node',
                    'vehicle_data_websocket_server', 'websocket_teleop_bridge'}
        names = set(self.get_node_names())
        missing = sorted(required - names)
        control_fresh = self.control_time is not None and now - self.control_time <= 3.0
        failures = [error['message'] for item in peripherals.values()
                    if item['state'] == 'error' for error in item['errors']]
        if self.pending_network_ack:
            ack, received = self.pending_network_ack
            if now - received > 5:
                self.pending_network_ack = None
            elif (state == 'runtime_fault' and not failures and not missing and control_fresh
                    and self.control.get('safety_state') == 'READY' and self.control.get('drive_state') == 0
                    and not self.control.get('estop_reason') and self.control.get('can_available')
                    and not self.control.get('last_send_error') and self.link_ok
                    and self.link_time is not None and now - self.link_time < 1.5
                    and ((ack.get('cleared_reason') in ('', 'drive_cmd') and self.control.get('last_paused_reset_request_id') == ack.get('request_id'))
                         or (ack.get('cleared_reason') in ('network_disconnected','network_heartbeat_timeout','network_unconfirmed')
                             and self.control.get('last_network_reset_request_id') == ack.get('request_id')
                             and self.control.get('last_network_reset_event_id') == ack.get('cleared_network_event_id')))
                    and all(p['state'] == 'ready' for p in peripherals.values())):
                lifecycle = update('ready', '', network_ack=dict(ack, boot_id=lifecycle['boot_id'], validated_current_reset=True, operator_reset=ack.get('cleared_reason') in ('', 'drive_cmd')))
                self.pending_network_ack = None
        if lifecycle.get('phase') == 'launching' and state == 'starting':
            elapsed = now - lifecycle.get('launch_at', now)
            if not failures and all(p['state'] == 'ready' for p in peripherals.values()) and not missing and control_fresh:
                if not self.control.get('can_available'):
                    failures.append('CAN 执行接口不可用')
                elif self.control.get('last_send_error'):
                    failures.append('CAN 发送失败：' + self.control['last_send_error'])
                else:
                    lifecycle = update('ready', '', phase='running')
            if failures or (elapsed > 15.0 and (missing or not control_fresh)):
                detail = failures or ['控制节点未就绪：' + ', '.join(missing or ['执行节点状态未更新'])]
                lifecycle = update('startup_failed', '；'.join(detail))
        elif state == 'ready':
            if missing:
                failures.append('控制节点退出：' + ', '.join(missing))
            if not control_fresh:
                failures.append('执行节点状态超时')
            elif self.control.get('last_send_error'):
                failures.append('CAN 发送失败：' + self.control['last_send_error'])
            elif not self.control.get('can_available'):
                failures.append('CAN 执行接口不可用')
            elif self.control.get('safety_state') == 'ESTOP':
                failures.append('车辆已急停/中止：' + self.control.get('estop_reason', ''))
            if failures:
                lifecycle = update('runtime_fault', '；'.join(failures), fault_event_id=self.control.get('network_fault_id'))
        if self.reset_guard_pub is not None:
            allowed = (not missing and control_fresh and self.control.get('can_available')
                       and not self.control.get('last_send_error')
                       and all(p['state'] == 'ready' for p in peripherals.values()))
            self.reset_guard_pub.publish(Bool(data=bool(allowed)))
        state = lifecycle['device_state']
        self.diagnostics.set_boot(lifecycle['boot_id'])
        emergency = fault_presentation(self.control, control_fresh, self.diagnostics.snapshot()['events'])
        self.snapshot = {'emergency_status': emergency,
            'device_state_label': ('紧急触发' if emergency['code'] == 'drive_cmd' else emergency['message']) if emergency['active'] else {'runtime_fault': '运行故障：'+lifecycle.get('device_message', ''), 'ready': '就绪', 'starting': '启动中', 'startup_failed': '启动失败：'+lifecycle.get('device_message', '')}.get(state, state),
            'type': 'device_status', 'version': 1,
            'boot_id': lifecycle['boot_id'], 'device_state': state,
            'device_message': lifecycle.get('device_message', ''),
            'vehicle_state': vehicle_state(self.control, self.link_ok and self.link_time is not None and now - self.link_time < 1.5) if control_fresh else None,
            'peripherals': peripherals,
            'network_status': summarize_network(self.control, control_fresh, self.link_ok,
                self.link_time is not None and now - self.link_time < 1.5, self.network_event),
            'diagnostics': self.diagnostics.snapshot(),
            'execution_status': dict(self.control, fresh=control_fresh),
            'obstacle_warning': summarize(self.obstacle_ranges, self.obstacle_received, now,
                                          self.args.runtime_timeout, self.obstacle_threshold,
                                          self.obstacle_warning_threshold)}

    def packet(self):
        if not self.snapshot:
            self.check_once()
        self.seq += 1
        return dict(self.snapshot, seq=self.seq,
                    timestamp=datetime.now(timezone.utc).isoformat())


async def run(args):
    rclpy.init()
    node = StatusService(args.startup_timeout, args.runtime_timeout, args.obstacle_threshold_m, args.obstacle_warning_threshold_m)
    bus_task = asyncio.create_task(poll_can_bus(node))
    marker = Path(str(args.state_file) + '.ready')
    async def handler(ws, path):
        if path != '/status':
            await ws.close(code=1008, reason='Use /status')
            return
        previous = None
        last_sent = 0.0
        while rclpy.ok() and not ws.closed:
            current = json.dumps(node.snapshot, ensure_ascii=False, sort_keys=True)
            now = time.monotonic()
            if current != previous or now - last_sent >= 1.0:
                await ws.send(json.dumps(node.packet(), ensure_ascii=False, allow_nan=False))
                previous, last_sent = current, now
            await asyncio.sleep(0.1)
    try:
        async with serve(handler, args.host, args.port, ping_interval=20, ping_timeout=10):
            marker.write_text(str(os.getpid()))
            node.check_once()
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                await asyncio.sleep(0.01)
    finally:
        bus_task.cancel()
        await asyncio.gather(bus_task, return_exceptions=True)
        marker.unlink(missing_ok=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=9011)
    parser.add_argument('--startup-timeout', type=float, default=5.0)
    parser.add_argument('--runtime-timeout', type=float, default=2.0)
    parser.add_argument('--obstacle-threshold-m', type=float, default=0.5, help='仅显示预警，不联动停车')
    parser.add_argument('--obstacle-warning-threshold-m', type=float, default=2.0, help='警告距离上限（含等于），仅显示')
    parser.add_argument('--state-file', default=os.environ.get('NAV_STATUS_FILE', '/tmp/nav_device_status.json'))
    args = parser.parse_args()
    import math
    if not math.isfinite(args.obstacle_threshold_m) or args.obstacle_threshold_m <= 0:
        parser.error("--obstacle-threshold-m 必须为正有限数值")
    if not math.isfinite(args.obstacle_warning_threshold_m) or args.obstacle_warning_threshold_m <= args.obstacle_threshold_m:
        parser.error("警告距离上限必须大于报警阈值")
    os.environ['NAV_STATUS_FILE'] = args.state_file
    # Keep module PATH consistent when overriding the path on the command line.
    import status_store
    status_store.PATH = Path(args.state_file)
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass
