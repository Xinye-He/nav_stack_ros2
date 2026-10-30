#!/usr/bin/env python3
"""Host-only, fixed-interface CAN inspection and single-shot BUS-OFF recovery."""
import argparse
import asyncio
from collections import OrderedDict
import json
import os
from pathlib import Path
import shutil
import time

INTERFACE = 'can0'
BASE = Path(__file__).resolve().parent


def decode_link(value):
    link = value[0]
    info = link.get('linkinfo', {}).get('info_data', {})
    stats = link.get('stats64', link.get('stats', {}))
    counters = info.get('berr_counter', {})
    return dict(interface=INTERFACE, interface_up='UP' in link.get('flags', []),
        state=info.get('state', 'UNKNOWN'), link_state=link.get('operstate', 'UNKNOWN'),
        bitrate=info.get('bittiming', {}).get('bitrate'),
        tx_error_counter=counters.get('tx'), rx_error_counter=counters.get('rx'),
        restart_ms=info.get('restart_ms'), rx_packets=stats.get('rx', {}).get('packets'),
        tx_packets=stats.get('tx', {}).get('packets'))


async def command(*args):
    proc = await asyncio.create_subprocess_exec(*args, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
    try:
        out, err = await asyncio.wait_for(proc.communicate(), 3)
    except asyncio.TimeoutError:
        proc.kill()
        await proc.wait()
        raise RuntimeError('ip 命令执行超时')
    if proc.returncode:
        raise RuntimeError(err.decode(errors='replace')[:600])
    return out.decode()


class Controller:
    def __init__(self, probe=None, restart=None):
        self.probe = probe or self.inspect
        self.restart = restart or self.restart_bus
        self.busy = False
        self.cache = OrderedDict()
        self.ip = shutil.which('ip') or '/sbin/ip'

    async def inspect(self):
        if not Path('/sys/class/net/' + INTERFACE).exists():
            return dict(interface=INTERFACE, state='MISSING', interface_up=False)
        try:
            return decode_link(json.loads(await command(self.ip, '-details', '-statistics', '-json', 'link', 'show', 'dev', INTERFACE)))
        except (ValueError, KeyError, IndexError, RuntimeError, OSError) as exc:
            return dict(interface=INTERFACE, state='UNKNOWN', interface_up=None, error=str(exc))

    async def restart_bus(self):
        # No shell, no user-controlled interface/bitrate/arguments, no down/up fallback.
        await command(self.ip, 'link', 'set', 'dev', INTERFACE, 'type', 'can', 'restart')

    async def execute(self, request):
        action, rid = request.get('action'), request.get('request_id')
        base = dict(type='can_result', action=action, request_id=rid)
        if action not in ('status', 'recover') or not isinstance(rid, str) or not 1 <= len(rid) <= 128:
            return dict(base, ok=False, code='INVALID_REQUEST', message='无效 CAN 操作或 request_id')
        if action == 'status':
            return dict(base, ok=True, code='OK', message='CAN 接口状态查询成功', bus=dict(await self.probe(), recovering=self.busy))
        if rid in self.cache:
            return self.cache[rid]
        if self.busy:
            return dict(base, ok=False, code='CAN_BUSY', message='CAN 恢复正在执行')
        proof = request.get('safety_proof')
        if (request.get('confirm_stopped') is not True or not isinstance(proof, dict)
                or proof.get('request_id') != rid or proof.get('mode') not in ('executor_paused', 'startup_failed_no_executor')
                or not isinstance(proof.get('checked_at'), (int, float))
                or not 0 <= time.monotonic() - proof['checked_at'] <= 2.0):
            return dict(base, ok=False, code='CAN_SAFETY_REJECTED', message='缺少新鲜的暂停维护确认或车辆停止确认')
        self.busy = True
        try:
            before = await self.probe()
            if before.get('state') == 'MISSING':
                result = dict(base, ok=False, code='CAN_INTERFACE_MISSING', message='CAN 接口 can0 不存在', bus=before)
            elif not before.get('interface_up'):
                result = dict(base, ok=False, code='CAN_INTERFACE_DOWN', message='can0 不存在或未启用；本接口不会自动配置波特率或启用总线', bus=before)
            elif before.get('state') == 'ERROR-ACTIVE':
                result = dict(base, ok=True, code='ALREADY_ACTIVE', message='接口已正常，无需复位', bus=before, interface_recovered=True)
            elif before.get('state') != 'BUS-OFF':
                result = dict(base, ok=False, code='CAN_NOT_BUS_OFF', message='接口状态不是 BUS-OFF，不执行复位', bus=before)
            else:
                await self.restart()
                await asyncio.sleep(0.15)
                after = await self.probe()
                good = after.get('state') == 'ERROR-ACTIVE' and after.get('interface_up') is True
                result = dict(base, ok=good, code='OK' if good else 'CAN_INTERFACE_NOT_RECOVERED',
                    message='CAN 接口已恢复，等待客户端验证 VCU 反馈' if good else 'CAN 接口未恢复或再次进入错误状态',
                    interface_recovered=good, bus=after)
        except (OSError, RuntimeError, ValueError) as exc:
            result = dict(base, ok=False, code='CAN_RECOVERY_FAILED', message=str(exc))
        finally:
            self.busy = False
        self.cache[rid] = result
        while len(self.cache) > 100:
            self.cache.popitem(last=False)
        return result


async def run(socket_path):
    path = Path(socket_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    os.chmod(path.parent, 0o770)
    if path.exists():
        try:
            reader, writer = await asyncio.open_unix_connection(str(path))
        except (ConnectionRefusedError, FileNotFoundError):
            path.unlink(missing_ok=True)
        else:
            writer.close()
            await writer.wait_closed()
            raise RuntimeError('CAN 管理服务已运行')
    controller = Controller()
    async def handler(reader, writer):
        try:
            request = json.loads(await asyncio.wait_for(reader.readline(), 3))
            if not isinstance(request, dict):
                raise ValueError('请求必须是 JSON 对象')
            response = await controller.execute(request)
            writer.write((json.dumps(response, ensure_ascii=False, allow_nan=False) + '\n').encode())
            await writer.drain()
        except (ValueError, asyncio.TimeoutError, OSError) as exc:
            try:
                writer.write((json.dumps(dict(type='can_result', ok=False, code='INVALID_REQUEST', message=str(exc))) + '\n').encode())
                await writer.drain()
            except OSError:
                pass
        finally:
            writer.close()
            await writer.wait_closed()
    server = await asyncio.start_unix_server(handler, path=str(path), limit=16384)
    os.chmod(path, 0o660)
    try:
        async with server:
            await server.serve_forever()
    finally:
        path.unlink(missing_ok=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--socket', default=str(BASE / '.runtime/control.sock'))
    args = parser.parse_args()
    try:
        asyncio.run(run(args.socket))
    except KeyboardInterrupt:
        pass
