"""Mode-aware network faults. The executor, not the client, decides to ESTOP."""
from collections import OrderedDict
import json
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String

NETWORK_REASONS = ('network_disconnected', 'network_heartbeat_timeout', 'network_unconfirmed')


def event_qos():
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    return qos


class ControlSessions:
    """Queries cannot own the control link or mask its missing heartbeat."""
    def __init__(self):
        self.clients = {}
        self.owner = None

    def connect(self, token, now):
        self.clients[token] = now

    def message(self, token, now, claims=False, heartbeat=False, timeout=1.5):
        previous_age = None if self.owner not in self.clients else now - self.clients[self.owner]
        self.clients[token] = now
        if claims or (heartbeat and (self.owner is None or previous_age is None or previous_age > timeout)):
            self.owner = token
        return self.owner == token

    def disconnect(self, token):
        owned = self.owner == token
        self.clients.pop(token, None)
        if owned:
            self.owner = None
        return owned


def fault_action(node):
    if node.estop_latched or node.abort_req or node.drive_state == node.DS_ESTOP:
        return 'warning', '已有急停保护，保留原保护原因'
    if node.remote_req:
        return 'warning', '实体遥控接管，网络断链仅告警'
    teleop = node.teleop_active and node.cmd_teleop is not None and node.cmd_teleop.valid
    bale = node.bale_active and node.cmd_bale is not None and node.cmd_bale.valid
    trajectory = node.cmd_traj is not None and node.cmd_traj.valid
    if not teleop and not bale and trajectory:
        return 'warning', '循迹模式网络断链，仅告警，循迹继续运行'
    if teleop and node.teleop_source == 'keyboard':
        return 'warning', '本地键盘控制，网络断链仅告警'
    if teleop and node.teleop_source == 'websocket_9010':
        return 'estop', '9010遥控网络断链，车辆已急停，重连后需手动复位'
    if node.drive_state == node.DS_PAUSED:
        return 'warning', '车辆已暂停，网络断链仅告警'
    return 'estop', '控制状态未明确确认为循迹，按断链急停处理'


class ExecutorNetworkSafety:
    def __init__(self, node):
        self.node = node
        self.active = False
        self.fault_id = None
        self.cache = OrderedDict()
        self.pub = node.create_publisher(String, '/safety/network_result', 10)
        node.create_subscription(String, '/safety/network_event', self.on_event, event_qos())

    def on_event(self, msg):
        try:
            event = json.loads(msg.data)
            rid = event['request_id']
            if not isinstance(rid, str) or not 1 <= len(rid) <= 128 or event.get('action') != 'fault':
                return
            reason = event['reason_code']
            if reason not in NETWORK_REASONS:
                return
        except (ValueError, TypeError, KeyError):
            return
        if rid in self.cache:
            self.pub.publish(String(data=json.dumps(self.cache[rid], ensure_ascii=False)))
            return
        node = self.node
        if self.active and not node.abort_req:
            action, message = 'estop', '网络急停仍在锁存，重连后需手动复位'
        else:
            action, message = fault_action(node)
        if action == 'estop':
            if not self.active:
                self.fault_id = rid
                node.last_network_reset_request_id = None
                node.last_network_reset_event_id = None
                node.estop_reason = reason
            self.active = True
            node.estop_latched = True
            node.drive_state = node.DS_ESTOP
            node.cmd_teleop = node._make_stop_command()
            node.teleop_active = True
            node.bale_active = False
            node.reset_estop_req = False
        result = dict(type='network_event', request_id=rid, action=action, message=message,
                      reason_code=reason, detail=event.get('detail', ''), source='stack_can_executor')
        self.cache[rid] = result
        while len(self.cache) > 50:
            self.cache.popitem(last=False)
        self.pub.publish(String(data=json.dumps(result, ensure_ascii=False)))
