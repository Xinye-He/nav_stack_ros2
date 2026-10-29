#!/usr/bin/env python3
"""Keyboard HMI simulator. No ROS dependency; run on a PC or on the Jetson."""
import argparse
import asyncio
from contextlib import asynccontextmanager, contextmanager, suppress
import json
import os
from pathlib import Path
import sys
import uuid

WORK_HELP = """
仅倾倒/捡拾测试：已请求PAUSE、全零指令、手动接管。不会发送RUN；r仅请求保持PAUSE的复位。
  j 倾倒开/关    k 捡拾开/关（两者互斥）
  r 暂停复位    c/空格 清除所有作业位    x 急停    ? 帮助    Esc/Ctrl+C 退出
前进、后退、转向及其他作业键均屏蔽。保持心跳，但PAUSE下是否执行作业由VCU决定。
"""

HELP = """
9010 上位机模拟客户端（按键立即发送，无需回车）
  m 手动接管       u 自动循迹       t 切换手动/自动
  g RUN            z PAUSE          x 急停           r 清零并请求复位
  w/4 前进4km/h    8 前进8km/h      s/1 后退4km/h    0 速度归零
  a/d 左右10°     q/e 左右20°      c/空格 速度和转角归零
  j 倾倒           k 捡拾           l 卸载           h 捡拾动作
  n 重载路径并进入零速遥控（随后按t启动循迹）
  f 任务完成       v 开启相机       b 关闭相机       p 查询相机
  ? 查看帮助       Esc/Ctrl+C 退出
速度为持续设定值，松开按键不会自动停车。普通车辆命令无成功回执，请结合9011状态判断。
退出控制连接会触发Jetson已有断联保护。心跳每0.5秒发送，包括等待相机执行期间。
"""


def emit(label, value):
    print(label + ' ' + (json.dumps(value, ensure_ascii=False) if not isinstance(value, str) else value), flush=True)


class KeyboardCommands:
    def __init__(self, work_only=False):
        self.path_version = None
        self.work_only = work_only
        self.reset()

    def reset(self):
        self.command = dict(pre_speed_kmh=0.0, angle_deg=0.0,
                            pick=False, unload=False, dump=False, pick_action=False)

    def message(self, key):
        if self.work_only and key not in ('j', 'k', 'c', ' ', 'x', 'r'):
            emit('[已屏蔽]', '仅允许 j/k、r暂停复位、c/空格清零、x急停。')
            return None
        if self.work_only and key == 'r':
            self.reset()
            return {'type': 'reset_estop_paused', 'request_id': 'paused-' + uuid.uuid4().hex}
        if self.work_only:
            self.command.update(pre_speed_kmh=0.0, angle_deg=0.0, unload=False, pick_action=False)
            if key in ('c', ' '):
                self.reset()
                return dict(type='cmd', **self.command)
        if key == 'n':
            self.reset()
            self.path_version = None
            return {'type': 'trajectory', 'action': 'reload', 'request_id': 'reload-' + uuid.uuid4().hex}
        if key == 't':
            self.reset()
            return {'type': 'trajectory', 'action': 'toggle', 'request_id': 'toggle-' + uuid.uuid4().hex}
        simple = {'m': {'type': 'active', 'value': True},
                  'u': {'type': 'active', 'value': False},
                  'g': {'type': 'drive', 'state': 1},
                  'z': {'type': 'drive', 'state': 0}, 'x': {'type': 'drive', 'state': 2},
                  'r': {'type': 'reset_estop'}, 'f': {'type': 'task_done'}}
        if key in simple:
            if key in ('x', 'r'):
                self.reset()
            return simple[key]
        if key in ('v', 'b', 'p'):
            return {'type': 'camera', 'action': {'v': 'start', 'b': 'stop', 'p': 'status'}[key],
                    'request_id': 'keyboard-' + uuid.uuid4().hex}
        speeds = {'w': 4.0, '4': 4.0, '8': 8.0, 's': -4.0, '1': -4.0, '0': 0.0}
        angles = {'a': -10.0, 'd': 10.0, 'q': -20.0, 'e': 20.0}
        if key in speeds:
            self.command['pre_speed_kmh'] = speeds[key]
        elif key in angles:
            self.command['angle_deg'] = angles[key]
        elif key in ('c', ' '):
            self.command.update(pre_speed_kmh=0.0, angle_deg=0.0)
        elif key in ('j', 'k', 'l', 'h'):
            field = {'j': 'dump', 'k': 'pick', 'l': 'unload', 'h': 'pick_action'}[key]
            if key in ('j', 'k') and self.command['pick_action']:
                emit('[提示]', '捡拾动作开启时不能切换捡拾/倾倒，请先按 h 关闭。')
                return None
            self.command[field] = not self.command[field]
            if self.command[field]:
                if key == 'j': self.command['pick'] = False
                if key == 'k': self.command['dump'] = False
                if key == 'h': self.command.update(pick=False, dump=False)
        else:
            return None
        return dict(type='cmd', **self.command)


@contextmanager
def terminal_queue():
    if not sys.stdin.isatty():
        raise RuntimeError('键盘模式需要交互终端。离线测试可用 --dry-run --keys "mgwc"。')
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    queue = asyncio.Queue()
    loop = asyncio.get_running_loop()
    def available():
        data = os.read(fd, 32)
        for key in data.decode(errors='ignore') if data else '\x04':
            queue.put_nowait(key)
    try:
        tty.setcbreak(fd)
        loop.add_reader(fd, available)
        yield queue
    finally:
        loop.remove_reader(fd)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def connect_api():
    try:
        try:
            from websockets.legacy.client import connect
        except ImportError:
            from websockets import connect
        @asynccontextmanager
        async def compatible_connect(url, open_timeout=5):
            # websockets 8.x has no open_timeout keyword; enforce it externally.
            ws = await asyncio.wait_for(connect(url), open_timeout)
            try:
                yield ws
            finally:
                with suppress(Exception):
                    await asyncio.wait_for(ws.close(), 3)
        return compatible_connect
    except ImportError:
        raise RuntimeError('缺少 websockets，请运行：python3 -m pip install websockets')


def status_summary(data):
    state_names = {'starting': '启动中', 'startup_failed': '启动失败', 'ready': '就绪', 'runtime_fault': '运行故障'}
    mode_names = {'trajectory': '循迹中', 'remote': '9010遥控模式', None: '空'}
    per_names = {'rtk': 'RTK', 'lidar': '雷达', 'ultrasonic': '超声波', 'can': 'CAN'}
    peripherals = []
    for name, item in data.get('peripherals', {}).items():
        state = {'starting': '启动中', 'ready': '就绪', 'error': '错误'}.get(item.get('state'), '未知')
        errors = '；'.join(e.get('message', '') for e in item.get('errors', []))
        peripherals.append(per_names.get(name, name) + ':' + state + ('(' + errors + ')' if errors else ''))
    return '{} | 模式:{} | {} | {}'.format(state_names.get(data.get('device_state'), '未知'),
        mode_names.get(data.get('vehicle_state'), '未知'), ' '.join(peripherals), data.get('device_message', ''))


async def receive_status(args, connect):
    url = 'ws://{}:{}/status'.format(args.host, args.status_port)
    previous = None
    while True:
        try:
            async with connect(url, open_timeout=5) as ws:
                emit('[状态连接]', url)
                while True:
                    # Prevent displaying a stale ready state if a socket stalls.
                    raw = await asyncio.wait_for(ws.recv(), 3)
                    data = json.loads(raw)
                    summary = status_summary(data)
                    if args.verbose or summary != previous:
                        emit('[9011]', data if args.verbose else summary)
                    previous = summary
        except Exception as exc:
            # websockets exceptions vary between supported versions.
            emit('[状态断线]', str(exc))
            previous = None
            await asyncio.sleep(2)


async def keyboard(queue, commands, send):
    while True:
        key = await queue.get()
        if key in ('\x1b', '\x04'):
            return
        if key == '?':
            print(WORK_HELP if commands.work_only else HELP, flush=True)
            continue
        msg = commands.message(key.lower())
        if msg is not None:
            await send(msg)


async def run(args):
    commands = KeyboardCommands(work_only=getattr(args, "work_only", True))
    if args.dry_run:
        emit('[离线模式]', '仅打印 JSON，不建立任何连接。')
        async def offline(msg): emit('[发送模拟]', msg)
        if commands.work_only:
            for msg in ({'type': 'drive', 'state': 0}, dict(type='cmd', **commands.command), {'type': 'active', 'value': True}):
                await offline(msg)
        if args.keys is not None:
            for key in args.keys:
                msg = commands.message(key.lower())
                if msg is not None: await offline(msg)
            return
        print(WORK_HELP if commands.work_only else HELP, flush=True)
        with terminal_queue() as queue:
            await keyboard(queue, commands, offline)
        return

    connect = connect_api()
    upload = args.upload_fence or args.upload_points
    if upload:
        kind = 'fence' if args.upload_fence else 'points'
        text = Path(upload).read_text(encoding='utf-8')
        url = 'ws://{}:{}/{}'.format(args.host, args.csv_port, kind)
        async with connect(url, open_timeout=5) as ws:
            await ws.send(text)
            response = await asyncio.wait_for(ws.recv(), 10)
            emit('[9002]', response)
            if not response.startswith('saved:'):
                raise RuntimeError('CSV 上传失败')
        return

    tasks = [asyncio.create_task(receive_status(args, connect))]
    try:
        if args.status_only:
            emit('[只读模式]', '仅连接9011，不连接9010，不发送控制指令。Ctrl+C退出。')
            await tasks[0]
            return
        # Fail before connecting if the caller does not have a keyboard terminal.
        with terminal_queue() as queue:
            url = 'ws://{}:{}'.format(args.host, args.control_port)
            async with connect(url, open_timeout=5) as ws:
                emit('[控制连接]', url)
                print(WORK_HELP if commands.work_only else HELP, flush=True)
                async def send(msg):
                    await ws.send(json.dumps(msg, ensure_ascii=False, allow_nan=False))
                    if args.verbose or msg.get('type') != 'heartbeat': emit('[9010发送]', msg)
                if commands.work_only:
                    # Stay paused and replace any old cached command before takeover.
                    await send({'type': 'drive', 'state': 0})
                    await send(dict(type='cmd', **commands.command))
                    await send({'type': 'active', 'value': True})
                async def heartbeat():
                    while True:
                        await send({'type': 'heartbeat'})
                        await asyncio.sleep(0.5)
                async def responses():
                    async for raw in ws:
                        try: value = json.loads(raw)
                        except ValueError: value = raw
                        if isinstance(value, dict) and value.get('type') == 'trajectory_result' and value.get('action') == 'reload':
                            commands.path_version = value.get('path_version') if value.get('ok') else None
                        emit('[9010回复]', value)
                    raise RuntimeError('9010 控制连接已断开；不会自动重连或重发运动指令。')
                controls = [asyncio.create_task(keyboard(queue, commands, send)),
                            asyncio.create_task(heartbeat()), asyncio.create_task(responses())]
                try:
                    done, _ = await asyncio.wait(controls, return_when=asyncio.FIRST_COMPLETED)
                    for task in done:
                        task.result()
                finally:
                    for task in controls: task.cancel()
                    await asyncio.gather(*controls, return_exceptions=True)
                    commands.reset()
                    with suppress(Exception):
                        await asyncio.wait_for(send(dict(type='cmd', **commands.command)), 1)
    finally:
        for task in tasks: task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter, epilog=WORK_HELP + "\n完整控制仅在 --full-control 下启用：\n" + HELP)
    parser.add_argument('--host', default='10.144.144.1')
    parser.add_argument('--control-port', type=int, default=9010)
    parser.add_argument('--status-port', type=int, default=9011)
    parser.add_argument('--csv-port', type=int, default=9002)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--status-only', action='store_true', help='只读状态，不连接远控端口')
    modes.add_argument('--dry-run', action='store_true', help='离线打印 JSON，不发送')
    modes.add_argument('--upload-fence', metavar='CSV', help='上传围栏 CSV 后退出（不连接9010）')
    modes.add_argument('--upload-points', metavar='CSV', help='上传路径 CSV 后退出（不连接9010）')
    controls = parser.add_mutually_exclusive_group()
    controls.add_argument('--work-only', dest='work_only', action='store_true', default=True, help='默认：仅倾倒/捡拾，强制零速零转向且保持暂停')
    controls.add_argument('--full-control', dest='work_only', action='store_false', help='显式启用原完整车辆控制按键')
    parser.add_argument('--keys', help='仅用于 dry-run，例如 mgwac')
    parser.add_argument('--verbose', action='store_true', help='显示心跳和完整状态 JSON')
    args = parser.parse_args()
    if args.keys is not None and not args.dry_run:
        parser.error('--keys 只能与 --dry-run 一起使用')
    return args


if __name__ == '__main__':
    try:
        asyncio.run(run(parse_args()))
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        emit('[错误]', str(exc))
        sys.exit(1)
