"""Nonblocking client for the host camera script runner."""
import asyncio
from contextlib import suppress
import json
import os
from pathlib import Path
import uuid


class CameraControlClient:
    def __init__(self):
        self.tasks = set()
        self.pending = False

    async def submit(self, websocket, data):
        request_id = data.get('request_id', str(uuid.uuid4()))
        action = data.get('action')
        base = {'type': 'camera_result', 'request_id': request_id, 'action': action}
        if action not in ('start', 'stop', 'status') or not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
            await websocket.send(json.dumps(dict(base, ok=False, code='INVALID_REQUEST', message='相机操作须为 start/stop/status，request_id 须为 1~128 字符'), ensure_ascii=False))
            return
        if self.pending:
            await websocket.send(json.dumps(dict(base, ok=False, code='CAMERA_BUSY', message='相机操作进行中，请稍后重试'), ensure_ascii=False))
            return
        self.pending = True
        task = asyncio.create_task(self._execute(websocket, dict(data, request_id=request_id), base))
        self.tasks.add(task)
        task.add_done_callback(self.tasks.discard)

    async def _execute(self, websocket, data, base):
        writer = None
        try:
            default = Path(os.environ.get('ROS_WS', '/root/stack_can_ws')) / 'script/video_usb/.runtime/control.sock'
            path = os.environ.get('CAMERA_CONTROL_SOCKET', str(default))
            reader, writer = await asyncio.wait_for(asyncio.open_unix_connection(path), 3)
            writer.write((json.dumps({'action': data['action'], 'request_id': data['request_id']}) + '\n').encode())
            await writer.drain()
            raw = await asyncio.wait_for(reader.readline(), 70)
            result = json.loads(raw)
        except (OSError, ValueError, asyncio.TimeoutError) as exc:
            result = dict(base, ok=False, code='CAMERA_SERVICE_UNAVAILABLE', message='宿主机相机服务不可用或执行超时：' + str(exc))
        finally:
            if writer:
                writer.close()
                with suppress(OSError):
                    await writer.wait_closed()
            self.pending = False
        try:
            await websocket.send(json.dumps(result, ensure_ascii=False))
        except Exception:
            # Camera operation completes even if the requesting client disconnects.
            pass
