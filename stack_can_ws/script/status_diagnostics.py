"""Bounded diagnostic log evidence; never infers the cause of a shared Bool abort."""
from collections import deque
from datetime import datetime, timezone


class DiagnosticEvents:
    def __init__(self, limit=30):
        self.events = deque(maxlen=limit)
        self.started_at = datetime.now(timezone.utc).isoformat()
        self.seq = 0
        self.boot_id = None

    def set_boot(self, boot_id):
        if self.boot_id is not None and self.boot_id != boot_id:
            self.events.clear()
            self.started_at = datetime.now(timezone.utc).isoformat()
        self.boot_id = boot_id

    def record(self, msg):
        source, text, level = msg.name, msg.msg, msg.level
        if level < 20:
            return
        rule = None
        if source == 'geofence_monitor' and ('Geofence:' in text or level >= 40):
            rule = ('GEOFENCE', 'trigger_report' if 'publish /abort = True' in text else 'related_log')
        elif source == 'motor_protection_node' and (level >= 30):
            rule = ('MOTOR_PROTECTION', 'trigger_report' if 'MOTOR PROTECTION ABORT:' in text else 'related_log')
        elif source == 'websocket_teleop_bridge' and ('NETWORK FAULT:' in text):
            rule = ('NETWORK_FAULT', 'trigger_report')
        elif source == 'stack_can_executor' and ('safety=ESTOP' in text or level >= 40):
            rule = ('EXECUTOR', 'related_log')
        elif level >= 40 and source in ('nmea_bridge','rsview','ultrasonic','can_feedback_node','traj_waypoint_follower'):
            rule = ('DRIVER_OR_CONTROL_ERROR', 'related_log')
        if rule is None:
            return
        text = text[:1200]
        # Periodic executor status should not evict the original trigger evidence.
        if any(e['source'] == source and e['message'] == text for e in self.events):
            return
        self.seq += 1
        stamp = getattr(msg, 'stamp', None)
        self.events.append(dict(event_id=self.seq, source=source, code=rule[0], evidence=rule[1],
            level={20:'INFO',30:'WARN',40:'ERROR',50:'FATAL'}.get(level,str(level)),
            message=text, received_at=datetime.now(timezone.utc).isoformat(),
            ros_stamp=dict(sec=stamp.sec,nanosec=stamp.nanosec) if stamp else None))

    def snapshot(self):
        return dict(coverage='since_status_service_start', started_at=self.started_at,
                    note='触发记录与相关日志，不保证是本次急停的唯一根因；历史事件不代表故障仍然存在',
                    events=list(self.events))
