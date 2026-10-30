"""Atomic lifecycle state shared by launcher, health check and status server."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import tempfile
import time
import uuid

PATH = Path(os.environ.get('NAV_STATUS_FILE', '/tmp/nav_device_status.json'))


def read():
    try:
        return json.loads(PATH.read_text())
    except FileNotFoundError:
        return {}


def update(state=None, message='', reset=False, network_ack=None, **extra):
    PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(str(PATH) + '.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        data = {} if reset else read()
        if not data:
            data = {'boot_id': str(uuid.uuid4()), 'started_at': time.monotonic(),
                    'ever_ready': False, 'device_state': 'starting'}
        acknowledged = False
        if network_ack is not None:
            reason = network_ack.get('cleared_reason')
            expected = '车辆已急停/中止：' + str(reason)
            acknowledged = (state == 'ready' and data.get('device_state') == 'runtime_fault'
                and reason in ('network_disconnected','network_heartbeat_timeout','network_unconfirmed')
                and data.get('boot_id') == network_ack.get('boot_id')
                and data.get('device_message') in tuple('车辆已急停/中止：'+r for r in ('network_disconnected','network_heartbeat_timeout','network_unconfirmed'))
                and network_ack.get('validated_current_reset') is True
                and isinstance(network_ack.get('cleared_network_event_id'), str))
            operator_ack = (state == 'ready' and data.get('device_state') == 'runtime_fault'
                and reason in ('drive_cmd', '')
                and data.get('device_message') == '车辆已急停/中止：drive_cmd'
                and data.get('boot_id') == network_ack.get('boot_id')
                and network_ack.get('validated_current_reset') is True
                and network_ack.get('operator_reset') is True)
            acknowledged = acknowledged or operator_ack
            if not acknowledged:
                return data
            data['acknowledged_fault'] = dict(message=data.get('device_message'), request_id=network_ack.get('request_id'), cleared_at=time.monotonic())
        # Preserve the first specific failure when the launch wrapper also exits.
        failed = data.get('device_state') in ('startup_failed', 'runtime_fault')
        if state and (not failed or state == 'starting' or reset or acknowledged):
            data.update(device_state=state, device_message=message)
            if state == 'starting':
                data['ever_ready'] = False
            if state == 'ready':
                data['ever_ready'] = True
        data.update(extra)
        data['updated_at'] = time.monotonic()
        fd, name = tempfile.mkstemp(prefix='.nav-status-', dir=str(PATH.parent))
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(data, stream, ensure_ascii=False)
            os.replace(name, PATH)
        finally:
            if os.path.exists(name):
                os.unlink(name)
        return data


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('state', choices=['starting', 'ready', 'startup_failed', 'runtime_fault'])
    parser.add_argument('message', nargs='?', default='')
    parser.add_argument('--reset', action='store_true')
    args = parser.parse_args()
    update(args.state, args.message, reset=args.reset)
