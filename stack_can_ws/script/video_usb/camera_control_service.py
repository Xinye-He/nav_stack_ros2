#!/usr/bin/env python3
"""Host-only camera script runner, reachable via a shared Unix socket."""
import argparse
import asyncio
from collections import OrderedDict
import json
import os
from pathlib import Path
import re
import signal
import time

BASE = Path(__file__).resolve().parent


async def command(*args, timeout=35):
    proc = await asyncio.create_subprocess_exec(*args, cwd=str(BASE),
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
        start_new_session=True)
    try:
        output, _ = await asyncio.wait_for(proc.communicate(), timeout)
        return proc.returncode, output.decode(errors='replace')[-4000:]
    except asyncio.TimeoutError:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            await asyncio.wait_for(proc.wait(), 3)
        except asyncio.TimeoutError:
            os.killpg(proc.pid, signal.SIGKILL)
            await proc.wait()
        return 124, '相机脚本执行超时'


async def config():
    if not (BASE / 'config.sh').is_file():
        raise ValueError('缺少 config.sh 相机配置')
    code, text = await command('bash', '-c',
        'source "$1"; test "${#CAM_NAMES[@]}" -eq "${#CAM_DEVS[@]}" || exit 2; '
        'printf "%s\\n" "$RTSP_SERVER" "$RTSP_PORT" "${#CAM_NAMES[@]}"; '
        'for i in "${!CAM_NAMES[@]}"; do printf "%s\\n" "${CAM_NAMES[$i]}" "${CAM_DEVS[$i]}" "${CAM_TYPES[$i]:-usb}"; done',
        'camera-config', str(BASE / 'config.sh'), timeout=3)
    lines = text.splitlines()
    if code or len(lines) < 3:
        raise ValueError('config.sh 格式无效或相机名称和设备数量不一致')
    count = int(lines[2])
    if not 1 <= count <= 8 or len(lines) != 3 + 3 * count:
        raise ValueError('相机配置数量无效')
    items = [{'name': lines[3+3*i], 'device': lines[4+3*i], 'kind': lines[5+3*i]} for i in range(count)]
    if any(not re.fullmatch(r'cam[0-9]+', c['name']) or not c['device'].startswith('/dev/') for c in items):
        raise ValueError('相机名称须为 cam数字，设备须位于 /dev')
    return lines[0], int(lines[1]), items


def camera_pid(name):
    try:
        pid = int((BASE / (name + '.pid')).read_text().strip())
        if pid <= 1:
            return None
        cmdline = Path('/proc/{}/cmdline'.format(pid)).read_bytes().replace(b'\0', b' ')
        if b'gst-launch-1.0' in cmdline and ('/' + name).encode() in cmdline:
            return pid
    except (OSError, ValueError):
        pass
    return None


async def stream_ready(host, port, name):
    writer = None
    try:
        reader, writer = await asyncio.wait_for(asyncio.open_connection(host, port), 1)
        request = 'DESCRIBE rtsp://{}:{}/{} RTSP/1.0\r\nCSeq: 1\r\nAccept: application/sdp\r\n\r\n'.format(host, port, name)
        writer.write(request.encode())
        await writer.drain()
        line = await asyncio.wait_for(reader.readline(), 1)
        return line.startswith(b'RTSP/1.0 200')
    except (OSError, asyncio.TimeoutError):
        return False
    finally:
        if writer:
            writer.close()
            await writer.wait_closed()


class CameraController:
    def __init__(self, public_host):
        self.public_host = public_host
        self.busy = False
        self.cache = OrderedDict()

    async def status(self):
        host, port, cameras = await config()
        code, text = await command('docker', 'inspect', '--format', '{{.State.Running}}', 'mediamtx', timeout=4)
        media_ok = code == 0 and text.strip() == 'true'
        for cam in cameras:
            cam['device_present'] = Path(cam['device']).exists()
            cam['process_running'] = camera_pid(cam['name']) is not None
            cam['stream_ready'] = media_ok and cam['process_running'] and await stream_ready(host, port, cam['name'])
            cam['url'] = 'rtsp://{}:{}/{}'.format(self.public_host, port, cam['name'])
        if media_ok and all(c['stream_ready'] for c in cameras):
            state = 'running'
        elif not media_ok and not any(c['process_running'] for c in cameras):
            state = 'stopped'
        else:
            state = 'partial'
        return {'camera_state': state, 'mediamtx_running': media_ok, 'cameras': cameras}

    @staticmethod
    def available_ready(status):
        cameras = [c for c in status['cameras'] if c.get('device_present', Path(c['device']).exists())]
        return bool(cameras) and all(c.get('stream_ready', False) for c in cameras)

    @staticmethod
    def start_result(result, status, **extra):
        ready = [c['name'] for c in status['cameras'] if c.get('stream_ready')]
        unavailable = [c['name'] for c in status['cameras'] if not c.get('stream_ready')]
        if not ready:
            return dict(result, ok=False, code='CAMERA_SCRIPT_FAILED',
                message='未检测到可播放视频流', **extra, **status)
        partial = bool(unavailable)
        return dict(result, ok=True, code='PARTIAL_READY' if partial else 'OK',
            message=('已启动：' + ', '.join(ready) + ('；未就绪：' + ', '.join(unavailable) if partial else '')),
            **extra, **status)

    async def execute(self, request):
        action = request.get('action')
        request_id = request.get('request_id')
        result = {'type': 'camera_result', 'request_id': request_id, 'action': action}
        if action not in ('start', 'stop', 'status') or not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
            return dict(result, ok=False, code='INVALID_REQUEST', message='action 须为 start/stop/status，request_id 须为 1~128 字符')
        if action != 'status' and request_id in self.cache:
            old_action, old_result = self.cache[request_id]
            if old_action != action:
                return dict(result, ok=False, code='REQUEST_ID_CONFLICT', message='同一 request_id 不能用于不同操作')
            return old_result
        if self.busy:
            return dict(result, ok=False, code='CAMERA_BUSY', message='相机启停操作进行中，请稍后重试')
        self.busy = True
        try:
            before = await self.status()
            if action == 'status':
                return dict(result, ok=True, code='OK', message='状态查询成功', **before)
            if action == 'stop' and before['camera_state'] == 'stopped':
                response = dict(result, ok=True, code='OK', message='已处于停止状态', **before)
            elif action == 'start' and self.available_ready(before):
                # A missing USB camera must not repeatedly restart a working CSI stream.
                response = self.start_result(result, before)
            else:
                if action == 'start':
                    available = [c for c in before['cameras'] if c.get('device_present', Path(c['device']).exists())]
                    if not available:
                        return dict(result, ok=False, code='CAMERA_DEVICE_MISSING', message='没有可用相机', **before)
                    if not (BASE / 'mediamtx.yml').is_file():
                        return dict(result, ok=False, code='CAMERA_CONFIG_MISSING', message='缺少 mediamtx.yml', **before)
                script = BASE / ('start_monitor_new.sh' if action == 'start' else 'stop_monitor.sh')
                code, output = await command('bash', str(script))
                after = await self.status()
                if action == 'start' and code == 0:
                    deadline = time.monotonic() + 5
                    while not self.available_ready(after) and time.monotonic() < deadline:
                        await asyncio.sleep(0.5)
                        after = await self.status()
                if action == 'start' and code == 0:
                    response = self.start_result(result, after, exit_code=code, output=output)
                else:
                    success = action == 'stop' and code == 0 and after['camera_state'] == 'stopped'
                    response = dict(result, ok=success, code='OK' if success else 'CAMERA_SCRIPT_FAILED',
                        message='相机已关闭' if success else '脚本执行失败或未达到目标状态',
                        exit_code=code, output=output, **after)
            self.cache[request_id] = (action, response)
            while len(self.cache) > 100:
                self.cache.popitem(last=False)
            return response
        except (OSError, ValueError, asyncio.TimeoutError) as exc:
            return dict(result, ok=False, code='CAMERA_SERVICE_ERROR', message=str(exc))
        finally:
            self.busy = False


async def run(args):
    sock = Path(args.socket)
    sock.parent.mkdir(parents=True, exist_ok=True)
    # Never unlink a live service socket. The launcher serializes startup.
    if sock.exists():
        try:
            reader, writer = await asyncio.open_unix_connection(str(sock))
        except (ConnectionRefusedError, FileNotFoundError):
            sock.unlink(missing_ok=True)
        else:
            writer.close()
            await writer.wait_closed()
            raise RuntimeError('相机控制服务已经运行')
    controller = CameraController(args.public_host)
    async def handler(reader, writer):
        try:
            raw = await asyncio.wait_for(reader.readline(), 5)
            value = json.loads(raw)
            if not isinstance(value, dict):
                raise ValueError('请求必须为 JSON 对象')
            result = await controller.execute(value)
            writer.write((json.dumps(result, ensure_ascii=False) + '\n').encode())
            await writer.drain()
        except (ValueError, asyncio.TimeoutError, ConnectionError) as exc:
            writer.write((json.dumps({'type': 'camera_result', 'ok': False, 'code': 'INVALID_REQUEST', 'message': str(exc)}) + '\n').encode())
        finally:
            writer.close()
            await writer.wait_closed()
    server = await asyncio.start_unix_server(handler, path=str(sock), limit=8192)
    os.chmod(sock, 0o660)
    try:
        async with server:
            await server.serve_forever()
    finally:
        sock.unlink(missing_ok=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--socket', default=str(BASE / '.runtime/control.sock'))
    parser.add_argument('--public-host', default='10.244.241.213')
    try:
        asyncio.run(run(parser.parse_args()))
    except KeyboardInterrupt:
        pass
