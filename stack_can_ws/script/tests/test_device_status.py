"""Run in the ROS environment; callbacks are injected, no CAN is opened."""
import asyncio
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace as Obj
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rclpy
from std_msgs.msg import String
import status_store
from status_store import update, read
from status_service import StatusService
from status_model import vehicle_state

REQUIRED = ['stack_can_executor', 'traj_waypoint_follower', 'geofence_monitor',
            'dr_odometry', 'motor_protection_node',
            'vehicle_data_websocket_server', 'websocket_teleop_bridge']

class StatusTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        status_store.PATH = Path(self.tmp.name) / 'status.json'
        update('starting', '', reset=True, phase='initializing')
        self.node = StatusService(5.0, 2.0)
        self.node.get_node_names = lambda: REQUIRED

    def tearDown(self):
        self.node.destroy_node()
        self.tmp.cleanup()

    def healthy(self):
        self.node.fix_cb(Obj(status=Obj(status=0), latitude=29.0, longitude=119.0))
        self.node.heading_cb(Obj(data=90.0))
        self.node.lidar_cb(Obj(width=1, height=1, data=b'1234'))
        self.node.ultrasonic_cb(Obj(data=[0.0, 1.0, 2.0, 3.0]))
        self.node.can_feedback_cb(Obj(rpm_left_valid=True, rpm_right_valid=False,
            throttle_left_valid=False, throttle_right_valid=False))
        self.node.on_control(String(data=json.dumps(dict(safety_state='READY',
            control_mode='TRAJ', drive_state=1, command_source='unknown',
            can_available=True, last_send_error=''))))

    def ready(self):
        self.healthy()
        update('starting', '', phase='launching', launch_at=time.monotonic())
        self.node.check_once()
        self.assertEqual(self.node.snapshot['device_state'], 'ready')

    def test_diagnostic_source_in_json_without_changing_health(self):
        self.ready()
        self.node.on_log(Obj(name='motor_protection_node',msg='MOTOR PROTECTION ABORT: left motor stalled',level=40,stamp=Obj(sec=10,nanosec=0)))
        self.node.check_once()
        packet=self.node.packet()
        event=packet['diagnostics']['events'][-1]
        self.assertEqual(event['source'],'motor_protection_node')
        self.assertEqual(event['evidence'],'trigger_report')
        self.assertIn('left motor stalled',event['message'])
        self.assertEqual(packet['device_state'],'ready')
        self.assertTrue(packet['execution_status']['fresh'])
        json.dumps(packet,allow_nan=False)

    def test_waiting_peripherals_and_startup_failure(self):
        self.node.check_once()
        self.assertEqual(self.node.snapshot['peripherals']['lidar']['state'], 'starting')
        self.node.start_time -= 6
        update('startup_failed', '外设检查失败')
        self.node.check_once()
        self.assertEqual(self.node.snapshot['device_state'], 'startup_failed')
        self.assertEqual(self.node.snapshot['peripherals']['lidar']['state'], 'error')

    def test_ready_and_actual_9010_mode(self):
        self.ready()
        self.assertEqual(self.node.snapshot['vehicle_state'], 'trajectory')
        self.node.control.update(control_mode='TELEOP', command_source='keyboard')
        self.node.check_once()
        self.assertIsNone(self.node.snapshot['vehicle_state'])
        self.node.control['command_source'] = 'websocket_9010'
        self.node.link_ok = True
        self.node.link_time = time.monotonic()
        self.node.check_once()
        self.assertEqual(self.node.snapshot['vehicle_state'], 'remote')

    def test_runtime_sensor_fault_is_latched_and_recovery_visible(self):
        self.ready()
        self.node.start_time -= 6
        self.node.last_lidar_time -= 3
        self.node.check_once()
        self.assertEqual(self.node.snapshot['device_state'], 'runtime_fault')
        self.assertIsNone(self.node.snapshot['vehicle_state'])
        self.healthy()
        self.node.check_once()
        self.assertEqual(self.node.snapshot['peripherals']['lidar']['state'], 'ready')
        self.assertEqual(self.node.snapshot['device_state'], 'runtime_fault')

    def test_executor_loss_and_can_send_error(self):
        self.ready()
        self.node.control_time -= 4
        self.node.check_once()
        self.assertEqual(self.node.snapshot['device_state'], 'runtime_fault')
        self.assertIn('执行节点状态超时', self.node.snapshot['device_message'])
        update('starting', '', phase='launching', launch_at=time.monotonic())
        self.healthy()
        self.node.control['last_send_error'] = 'bus off'
        self.node.check_once()
        self.assertEqual(self.node.snapshot['device_state'], 'startup_failed')

    def test_no_ready_before_required_nodes_exist(self):
        self.healthy()
        self.node.get_node_names = lambda: []
        update('starting', '', phase='launching', launch_at=time.monotonic()-16)
        self.node.check_once()
        self.assertEqual(self.node.snapshot['device_state'], 'startup_failed')

    def test_invalid_heading_empty_cloud_and_driver_error(self):
        self.node.start_time -= 6
        self.healthy()
        self.node.heading_cb(Obj(data=float('nan')))
        self.node.lidar_cb(Obj(width=0, height=0, data=b''))
        self.node.on_log(Obj(level=40, name='rsview', msg='bind: address already in use'))
        self.node.check_once()
        self.assertEqual(self.node.snapshot['peripherals']['rtk']['state'], 'error')
        errors = self.node.snapshot['peripherals']['lidar']['errors']
        self.assertTrue(any(e['code'] == 'DRIVER_ERROR' for e in errors))
        json.dumps(self.node.packet(), allow_nan=False)

    def test_boot_reset_and_first_failure_preserved(self):
        old = read()['boot_id']
        update('runtime_fault', '雷达数据超时')
        update('runtime_fault', '进程退出')
        self.assertEqual(read()['device_message'], '雷达数据超时')
        update('starting', '', reset=True)
        self.assertNotEqual(old, read()['boot_id'])

    def test_executor_source_is_bound_to_command_callback(self):
        from main.stack_can_executor import StackCanExecutor
        executor = object.__new__(StackCanExecutor)
        cmd = object()
        executor._on_websocket_cmd(cmd)
        self.assertIs(executor.cmd_teleop, cmd)
        self.assertEqual(executor.teleop_source, 'websocket_9010')
        executor._on_teleop_cmd(cmd)
        self.assertEqual(executor.teleop_source, 'keyboard')

    def test_obstacle_warning_never_changes_device_health(self):
        self.ready()
        self.node.ultrasonic_cb(Obj(data=[0.2, 1.0, 2.0, 3.0]))
        self.node.check_once()
        self.assertTrue(self.node.snapshot['obstacle_warning']['warning'])
        self.assertEqual(self.node.snapshot['device_state'], 'ready')
        self.assertEqual(self.node.snapshot['peripherals']['ultrasonic']['errors'], [])
        self.node.ultrasonic_cb(Obj(data=[0.0, 0.0, 0.0, 0.0]))
        self.node.check_once()
        self.assertIsNone(self.node.snapshot['obstacle_warning']['warning'])
        self.assertEqual(self.node.snapshot['device_state'], 'ready')

    def test_websocket_snapshot_reconnect_and_wrong_path(self):
        async def probe():
            from websockets.legacy.client import connect
            env = dict(os.environ, NAV_STATUS_FILE=str(status_store.PATH))
            script = Path(__file__).resolve().parents[1] / 'status_service.py'
            proc = subprocess.Popen([sys.executable, str(script), '--host', '127.0.0.1',
                '--port', '19011'], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                for attempt in range(100):
                    if Path(str(status_store.PATH) + '.ready').exists():
                        break
                    if proc.poll() is not None:
                        self.fail('Status service exited')
                    await asyncio.sleep(0.1)
                async with connect('ws://127.0.0.1:19011/status') as ws:
                    packet = json.loads(await asyncio.wait_for(ws.recv(), 3))
                    self.assertEqual(packet['device_state'], 'starting')
                    update('startup_failed', '测试外设失败')
                    for attempt in range(10):
                        packet = json.loads(await asyncio.wait_for(ws.recv(), 3))
                        if packet['device_state'] == 'startup_failed':
                            break
                    self.assertEqual(packet['device_state'], 'startup_failed')
                async with connect('ws://127.0.0.1:19011/status') as ws:
                    self.assertEqual(json.loads(await ws.recv())['device_state'], 'startup_failed')
                async with connect('ws://127.0.0.1:19011/wrong') as ws:
                    await ws.wait_closed()
                    self.assertEqual(ws.close_code, 1008)
            finally:
                proc.terminate()
                proc.wait(timeout=10)
        asyncio.run(probe())

if __name__ == '__main__':
    unittest.main(verbosity=2)
