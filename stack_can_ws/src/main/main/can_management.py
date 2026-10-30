"""ROS-side maintenance interlock and nonblocking CAN host-service client."""
import asyncio
from collections import OrderedDict
from contextlib import suppress
import json
import math
import os
from pathlib import Path
import time
import uuid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
from stack_msgs.msg import CanFeedback


def maintenance_qos():
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    return qos


async def host_request(request):
    path = os.environ.get('CAN_CONTROL_SOCKET', str(Path(os.environ.get('ROS_WS', '/root/stack_can_ws')) / 'script/can_control/.runtime/control.sock'))
    writer = None
    try:
        reader, writer = await asyncio.wait_for(asyncio.open_unix_connection(path), 2)
        writer.write((json.dumps(request, allow_nan=False) + '\n').encode())
        await writer.drain()
        result = json.loads(await asyncio.wait_for(reader.readline(), 6))
        if not isinstance(result, dict) or result.get('request_id') != request['request_id'] or result.get('action') != request['action']:
            raise ValueError('宿主机 CAN 回执不匹配')
        return result
    finally:
        if writer:
            writer.close()
            with suppress(OSError):
                await writer.wait_closed()


class ExecutorCanMaintenance:
    def __init__(self, node):
        self.node = node
        self.active = False
        self.request_id = None
        node.create_subscription(String, '/can/maintenance', self.on_request, maintenance_qos())

    def on_request(self, msg):
        try:
            request = json.loads(msg.data)
            rid = request['request_id']
            if not isinstance(rid, str) or not 1 <= len(rid) <= 128 or not isinstance(request.get('active'), bool):
                return
        except (ValueError, TypeError, KeyError):
            return
        if request['active']:
            self.active = True
            self.request_id = rid
            self.pause()
        elif self.active and rid == self.request_id:
            self.pause()
            self.active = False

    def pause(self):
        node = self.node
        node.drive_state = node.DS_ESTOP if node.estop_latched or node.abort_req else node.DS_PAUSED
        node.reset_estop_req = False
        node.cmd_teleop = node._make_stop_command()
        node.teleop_active = True
        node.bale_active = False
        node.teleop_source = 'websocket_9010'


def recovery_rejection(execution, execution_at, feedback, feedback_at, now):
    if execution_at is not None and now - execution_at <= 2:
        if execution.get('remote_req'):
            return '实体遥控正在接管，禁止恢复 CAN'
        if execution.get('drive_state') not in (0, 2):
            return '车辆尚未暂停，请先 PAUSE 或急停，再请求恢复 CAN'
    if feedback is not None and feedback_at is not None and now - feedback_at <= 1:
        valid = feedback.rpm_left_valid or feedback.rpm_right_valid
        speeds = [float(feedback.ground_speed_mps)]
        if feedback.rpm_left_valid:
            speeds.append(float(getattr(feedback, 'left_wheel_speed_mps', 0.0)))
        if feedback.rpm_right_valid:
            speeds.append(float(getattr(feedback, 'right_wheel_speed_mps', 0.0)))
        if valid and any(not math.isfinite(v) or abs(v) > 0.05 for v in speeds):
            return '有效 CAN 反馈显示车辆仍在移动，禁止恢复 CAN'
    return ''


class CanControlClient:
    def __init__(self, node):
        self.node = node
        self.pending = False
        self.execution, self.execution_at = {}, None
        self.feedback, self.feedback_at = None, None
        self.cache = OrderedDict()
        self.tasks = set()
        self.maintenance_rid = None
        self.pub = node.create_publisher(String, '/can/maintenance', maintenance_qos())
        node.create_subscription(String, '/vehicle/execution_status', self.on_execution, 10)
        node.create_subscription(CanFeedback, '/stack_can/feedback', self.on_feedback, 10)

    def on_execution(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self.execution, self.execution_at = value, time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_feedback(self, msg):
        self.feedback, self.feedback_at = msg, time.monotonic()

    def maintenance(self, rid, active):
        self.maintenance_rid = rid if active else None
        self.pub.publish(String(data=json.dumps(dict(request_id=rid, active=active))))

    def zero_and_pause(self):
        self.node.zero_teleop()
        # Do not clear an existing ESTOP selection.
        if self.node.drive_state != 2:
            self.node.drive_state = 0
        self.node.publish_drive_cmd()
        self.node.teleop_active = True
        self.node.publish_teleop_active()
        self.node.publish_teleop_cmd()

    async def safe_proof(self, rid):
        now = time.monotonic()
        reason = recovery_rejection(self.execution, self.execution_at, self.feedback, self.feedback_at, now)
        if reason:
            raise ValueError(reason)
        self.zero_and_pause()
        self.maintenance(rid, True)
        deadline, first_absent = now + 3.5, None
        while time.monotonic() < deadline:
            now = time.monotonic()
            reason = recovery_rejection(self.execution, self.execution_at, self.feedback, self.feedback_at, now)
            if reason:
                raise ValueError(reason)
            if self.execution_at is not None and now - self.execution_at <= 1.5:
                if (self.execution.get('can_maintenance_active') is True
                        and self.execution.get('can_maintenance_request_id') == rid
                        and self.execution.get('drive_state') in (0, 2)):
                    return dict(mode='executor_paused', request_id=rid, checked_at=now)
            # Startup failure is the one allowed case with no executor. A durable
            # maintenance request also gates an executor that joins during recovery.
            names = self.node.get_node_names()
            if 'stack_can_executor' not in names:
                if first_absent is None:
                    first_absent = now
                path = Path(os.environ.get('NAV_STATUS_FILE', '/tmp/nav_device_status.json'))
                try:
                    lifecycle = json.loads(path.read_text())
                except (OSError, ValueError):
                    lifecycle = {}
                if (now - first_absent >= 1.0 and lifecycle.get('device_state') == 'startup_failed'
                        and lifecycle.get('ever_ready') is False):
                    return dict(mode='startup_failed_no_executor', request_id=rid, checked_at=now)
            else:
                first_absent = None
            await asyncio.sleep(0.1)
        raise ValueError('执行器未确认暂停维护；请确认已加载新版节点，且当前未在运行控制程序')

    async def submit(self, websocket, data):
        rid = data.get('request_id', str(uuid.uuid4()))
        action = data.get('action')
        base = dict(type='can_result', action=action, request_id=rid)
        if action not in ('status', 'recover') or not isinstance(rid, str) or not 1 <= len(rid) <= 128:
            await websocket.send(json.dumps(dict(base, ok=False, code='INVALID_REQUEST', message='CAN 操作须为 status/recover，request_id 须为1~128字符'), ensure_ascii=False))
            return
        if action == 'recover' and data.get('confirm_stopped') is not True:
            await websocket.send(json.dumps(dict(base, ok=False, code='CAN_STOP_CONFIRMATION_REQUIRED', message='请先确认车辆已停止，并发送 confirm_stopped=true'), ensure_ascii=False))
            return
        if action == 'recover' and self.pending:
            await websocket.send(json.dumps(dict(base, ok=False, code='CAN_BUSY', message='CAN 恢复进行中'), ensure_ascii=False))
            return
        if action == 'recover' and rid in self.cache:
            await websocket.send(json.dumps(self.cache[rid], ensure_ascii=False))
            return
        if action == 'recover':
            self.pending = True
        task = asyncio.create_task(self.execute(websocket, data, base))
        self.tasks.add(task)
        task.add_done_callback(self.tasks.discard)

    async def execute(self, websocket, data, base):
        action, rid = base['action'], base['request_id']
        maintenance_entered = False
        try:
            request = dict(action=action, request_id=rid)
            if action == 'recover':
                # Release our interlock even if safe_proof raises after publishing it.
                maintenance_entered = True
                request['safety_proof'] = await self.safe_proof(rid)
                request['confirm_stopped'] = True
            result = await host_request(request)
            if action == 'recover' and result.get('ok'):
                since, deadline = time.monotonic(), time.monotonic() + 5.0
                valid = False
                while time.monotonic() < deadline:
                    msg = self.feedback
                    valid = (msg is not None and self.feedback_at is not None and self.feedback_at > since
                             and time.monotonic() - self.feedback_at <= 1.0
                             and (msg.rpm_left_valid or msg.rpm_right_valid or msg.throttle_left_valid or msg.throttle_right_valid))
                    if valid:
                        break
                    await asyncio.sleep(0.1)
                # Query again: an apparently successful restart may immediately BUS-OFF.
                latest = await host_request(dict(action='status', request_id=rid + '-verify' if len(rid) <= 120 else rid))
                bus = latest.get('bus', {})
                good_bus = bus.get('state') == 'ERROR-ACTIVE' and bus.get('interface_up') is True
                result.update(bus=bus, interface_recovered=good_bus, feedback_valid=bool(valid), ok=bool(valid and good_bus),
                    code='OK' if valid and good_bus else 'CAN_FEEDBACK_NOT_RECOVERED' if good_bus else 'CAN_INTERFACE_NOT_RECOVERED',
                    message='can0 已恢复，已收到有效 VCU 反馈；车辆保持暂停，未解除急停' if valid and good_bus
                    else '接口已恢复，但未收到有效 VCU 反馈，请检查供电、接线和波特率' if good_bus
                    else 'CAN 接口未恢复或再次进入错误状态')
        except ValueError as exc:
            result = dict(base, ok=False, code='CAN_SAFETY_REJECTED', message=str(exc))
        except (OSError, asyncio.TimeoutError) as exc:
            result = dict(base, ok=False, code='CAN_SERVICE_UNAVAILABLE', message='宿主机 CAN 服务不可用或超时：' + str(exc))
        finally:
            if action == 'recover':
                if maintenance_entered and self.maintenance_rid == rid:
                    self.zero_and_pause()
                    self.maintenance(rid, False)
                self.pending = False
        if action == 'recover':
            self.cache[rid] = result
            while len(self.cache) > 100:
                self.cache.popitem(last=False)
        try:
            await websocket.send(json.dumps(result, ensure_ascii=False, allow_nan=False))
        except Exception:
            pass
