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


def update(state=None, message='', reset=False, **extra):
    PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(str(PATH) + '.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        data = {} if reset else read()
        if not data:
            data = {'boot_id': str(uuid.uuid4()), 'started_at': time.monotonic(),
                    'ever_ready': False, 'device_state': 'starting'}
        # Preserve the first specific failure when the launch wrapper also exits.
        failed = data.get('device_state') in ('startup_failed', 'runtime_fault')
        if state and (not failed or state == 'starting' or reset):
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
