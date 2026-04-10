#!/usr/bin/env python3
import asyncio
import json
import time
import sys
import os
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8, String
from stack_msgs.msg import StackCommand
import websockets

HELP = """
websocket_teleop_bridge: WebSocket teleop + mode control

全局行驶状态 (/drive_cmd):
  x -> ESTOP (drive_cmd = 2)
  r -> RESET ESTOP / abort
       行为: 先发布零速零角 teleop 命令，再请求 /reset_estop，
            同时切回 RUN + 手动 teleop（但仍保持静止）
  f -> TASK DONE (/task_done True)
  t -> TOGGLE 自动循迹 / 手动 teleop
       注意: t 不再负责退出 ESTOP

teleop_active = True  -> 手动 teleop (通过 /stack_cmd/teleop 控制)
teleop_active = False -> 自动循迹 (跟随 /stack_cmd/traj)

Teleop 速度 (pre_speed_kmh)：
  0   -> 0 km/h
  4/w -> 4 km/h
  8   -> 8 km/h
  1/s -> -4 km/h (后退)

Teleop 方向 (angle_deg)：
  a -> -10 deg (小左)
  d -> +10 deg (小右)
  q -> -20 deg (大左)
  e -> +20 deg (大右)
  c -> speed=0, angle=0

作业位 (仅影响 teleop 的 StackCommand)：
  j -> TOGGLE dump
  k -> TOGGLE pick
  l -> TOGGLE unload
  h -> TOGGLE pick_action

- 本节点既支持 WebSocket 输入，也支持本地终端按键输入
"""

DEBOUNCE_SEC = 0.15


class WebSocketTeleopBridge(Node):
    def __init__(self):
        super().__init__('websocket_teleop_bridge')

        # 发布者
        self.pub_drive_cmd = self.create_publisher(UInt8, '/drive_cmd', 1)
        self.pub_task_done = self.create_publisher(Bool, '/task_done', 1)
        self.pub_teleop_act = self.create_publisher(Bool, '/teleop_active', 1)
        self.pub_teleop_cmd = self.create_publisher(StackCommand, '/stack_cmd/teleop', 1)
        self.pub_reset_estop = self.create_publisher(Bool, '/reset_estop', 1)
        self.pub_abort = self.create_publisher(Bool, '/abort', 1)
        self.pub_link_ok = self.create_publisher(Bool, '/network_link_ok', 1)
        self.pub_fault_text = self.create_publisher(String, '/network_fault_text', 10)

        # 内部状态
        self.drive_state = 1
        self.teleop_active = True

        self.pre_speed_kmh = 0.0
        self.angle_deg = 0.0
        self.pick = False
        self.unload = False
        self.dump = False
        self.pick_action = False

        self.last_ts = {}

        self.ws_connected = False
        self.active_client_addr = None
        self.last_rx_monotonic = 0.0

        self.declare_parameter('link_timeout_sec', 1.5)
        self.declare_parameter('abort_on_disconnect', True)

        self.link_timeout_sec = float(self.get_parameter('link_timeout_sec').value)
        self.abort_on_disconnect = bool(self.get_parameter('abort_on_disconnect').value)

        self.network_fault_latched = False
        self.last_link_ok_pub = None
        self.last_fault_text = ''

        self.create_timer(0.1, self.link_watchdog_loop)   # 10Hz

        # 本地终端相关
        self.fd = None
        self.raw_mode = False
        self.old_term = None
        try:
            self.fd = os.open('/dev/tty', os.O_RDWR | os.O_NOCTTY)
        except Exception:
            try:
                self.fd = sys.stdin.fileno()
            except Exception:
                self.fd = None

        self.has_tty = (self.fd is not None and os.isatty(self.fd))
        self.enable_raw()

        self.get_logger().info(HELP)

        # 初始广播一次
        self.publish_drive_cmd()
        self.publish_teleop_active()
        self.publish_teleop_cmd()

    # ---------------- 终端 raw mode ----------------
    def enable_raw(self):
        if not self.has_tty:
            self.get_logger().warn('No TTY available: local keyboard input disabled, websocket still works.')
            return False
        try:
            self.old_term = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            self.raw_mode = True
            self.get_logger().info('Local keyboard input enabled.')
            return True
        except Exception as e:
            self.get_logger().warn(f'Failed to enable raw mode: {e}')
            return False

    def disable_raw(self):
        if self.raw_mode and self.old_term is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)
            except Exception:
                pass
            self.raw_mode = False

    # ---------------- 工具 ----------------
    def debounce(self, key: str) -> bool:
        now = time.time()
        last = self.last_ts.get(key, 0.0)
        if (now - last) < DEBOUNCE_SEC:
            return False
        self.last_ts[key] = now
        return True

    def zero_teleop(self):
        self.pre_speed_kmh = 0.0
        self.angle_deg = 0.0
        self.pick = False
        self.unload = False
        self.dump = False
        self.pick_action = False

    #发布链路状态
    def publish_link_ok(self, ok: bool):
        if self.last_link_ok_pub is ok:
            return
        self.pub_link_ok.publish(Bool(data=bool(ok)))
        self.last_link_ok_pub = ok
        self.get_logger().info(f'/network_link_ok -> {ok}')

    #发布故障文本
    def publish_fault_text(self, text: str):
        if text == self.last_fault_text:
            return
        self.pub_fault_text.publish(String(data=text))
        self.last_fault_text = text
        self.get_logger().warn(text)
    
    #发布abort
    def publish_abort(self):
        self.pub_abort.publish(Bool(data=True))
        self.get_logger().warn('/abort -> True')
    
    #统一的“网络故障触发abort”入口
    def trigger_network_abort(self, reason: str):
        # 已经锁存过一次，就不要反复刷屏
        if self.network_fault_latched:
            return

        self.network_fault_latched = True

        # 为了避免锁存瞬间仍残留旧 teleop 命令，先清零再 abort
        self.zero_teleop()
        self.publish_teleop_cmd()

        self.publish_link_ok(False)
        self.publish_fault_text(f'NETWORK FAULT: {reason}')
        if self.abort_on_disconnect:
            self.publish_abort()

    #链路恢复入口 
    def mark_link_restored(self, client_addr=None):
        self.ws_connected = True
        self.active_client_addr = client_addr
        self.last_rx_monotonic = time.monotonic()
        self.publish_link_ok(True)

        # 注意：这里只报告恢复，不自动 clear abort
        if self.network_fault_latched:
            self.publish_fault_text(
                'NETWORK RECOVERED: link restored, manual reset_estop required'
            )
        else:
            self.publish_fault_text('NETWORK OK')

    #看门狗链路
    def link_watchdog_loop(self):
        now = time.monotonic()

        # 没客户端连接，且之前不是故障状态时，不一定要强制 abort；
        # 但如果你希望“远程模式下必须有链路”，这里可以直接判故障。
        if not self.ws_connected:
            return

        dt = now - self.last_rx_monotonic
        if dt > self.link_timeout_sec:
            self.ws_connected = False
            self.trigger_network_abort(
                f'websocket heartbeat timeout: no message for {dt:.2f}s'
            )

    # ---------------- 发布函数 ----------------
    def publish_drive_cmd(self):
        msg = UInt8()
        msg.data = int(self.drive_state)
        self.pub_drive_cmd.publish(msg)
        self.get_logger().info(f"/drive_cmd -> {self.drive_state}")

    def publish_task_done(self):
        msg = Bool()
        msg.data = True
        self.pub_task_done.publish(msg)
        self.get_logger().info("TASK DONE sent")

    def publish_teleop_active(self):
        msg = Bool()
        msg.data = bool(self.teleop_active)
        self.pub_teleop_act.publish(msg)
        self.get_logger().info(
            f"/teleop_active -> {self.teleop_active} "
            "(True=手动 teleop, False=自动循迹)"
        )

    def publish_reset_estop(self):
        msg = Bool()
        msg.data = True
        self.pub_reset_estop.publish(msg)
        self.get_logger().warn("/reset_estop -> True")

    def publish_teleop_cmd(self):
        msg = StackCommand()
        msg.pre_speed_kmh = float(self.pre_speed_kmh)
        msg.angle_deg = float(self.angle_deg)
        msg.dist_to_target_m = 0.0
        msg.pick = bool(self.pick)
        msg.unload = bool(self.unload)
        msg.dump = bool(self.dump)
        msg.pick_action = bool(self.pick_action)
        msg.valid = True
        self.pub_teleop_cmd.publish(msg)
        self.get_logger().info(
            "TELEOP CMD: "
            f"speed={self.pre_speed_kmh} km/h, angle={self.angle_deg} deg, "
            f"pick={self.pick}, unload={self.unload}, "
            f"dump={self.dump}, pick_action={self.pick_action}"
        )

    def request_reset_estop(self):
        self.zero_teleop()
        self.publish_teleop_cmd()

        self.teleop_active = True
        self.publish_teleop_active()

        self.drive_state = 1
        self.publish_drive_cmd()
        
        self.network_fault_latched = False
        self.publish_reset_estop()

    # ---------------- 按键语义 ----------------
    def handle_key(self, c: str):
        if not c:
            return
        c = c.lower()

        if c == 'x':
            if not self.debounce(c):
                return
            self.drive_state = 2
            self.publish_drive_cmd()
            return

        if c == 'r':
            if not self.debounce(c):
                return
            self.request_reset_estop()
            return

        if c == 'f':
            if not self.debounce(c):
                return
            self.publish_task_done()
            return

        if c == 't':
            if not self.debounce(c):
                return
            if self.drive_state == 2:
                self.get_logger().warn('Currently ESTOP. Use key "r" to reset ESTOP/abort first.')
                return
            self.teleop_active = not self.teleop_active
            self.publish_teleop_active()
            return

        if c in ('0', '1', '4', '8', 'w', 's'):
            if not self.debounce(c):
                return
            if c == '0':
                self.pre_speed_kmh = 0.0
            elif c in ('4', 'w'):
                self.pre_speed_kmh = 4.0
            elif c == '8':
                self.pre_speed_kmh = 8.0
            elif c in ('1', 's'):
                self.pre_speed_kmh = -4.0
            self.publish_teleop_cmd()
            return

        if c == 'c':
            if not self.debounce(c):
                return
            self.pre_speed_kmh = 0.0
            self.angle_deg = 0.0
            self.publish_teleop_cmd()
            return

        if c in ('a', 'd', 'q', 'e'):
            if not self.debounce(c):
                return
            if c == 'a':
                self.angle_deg = -10.0
            elif c == 'd':
                self.angle_deg = 10.0
            elif c == 'q':
                self.angle_deg = -20.0
            elif c == 'e':
                self.angle_deg = 20.0
            self.publish_teleop_cmd()
            return

        if c in ('j', 'k', 'l', 'h'):
            if not self.debounce(c):
                return

            if c == 'h':
                self.pick_action = not self.pick_action

                # h 生效时，j/k 全部清零
                if self.pick_action:
                    self.pick = False
                    self.dump = False

            elif c == 'j':
                # h 生效时，j 无效
                if self.pick_action:
                    self.get_logger().warn('pick_action is active, key "j" ignored')
                    return

                # j / k 互斥
                self.dump = not self.dump
                if self.dump:
                    self.pick = False

            elif c == 'k':
                # h 生效时，k 无效
                if self.pick_action:
                    self.get_logger().warn('pick_action is active, key "k" ignored')
                    return

                # k / j 互斥
                self.pick = not self.pick
                if self.pick:
                    self.dump = False

            elif c == 'l':
                self.unload = not self.unload

            self.publish_teleop_cmd()
            return

        self.get_logger().warn(f"Unknown key: {c}")

    # ---------------- WebSocket 处理 ----------------
    async def handle_client(self, websocket, path):
        client_addr = websocket.remote_address
        self.get_logger().info(f"Client connected from {client_addr}")

        self.mark_link_restored(client_addr)

        try:
            async for message in websocket:
                self.last_rx_monotonic = time.monotonic()

                try:
                    data = json.loads(message)
                except json.JSONDecodeError:
                    self.get_logger().warn(f"Invalid JSON from {client_addr}: {message}")
                    continue

                if 'key' in data:
                    key = str(data['key'])[:1]
                    self.handle_key(key)
                    continue

                msg_type = data.get('type')

                if msg_type == 'task_done':
                    if self.debounce('task_done'):
                        self.publish_task_done()
                    continue

                if msg_type == 'drive':
                    state = data.get('state')
                    if state in (0, 1, 2):
                        self.drive_state = int(state)
                        self.publish_drive_cmd()
                    else:
                        self.get_logger().warn(f"Invalid drive state: {state}")
                    continue

                if msg_type == 'reset_estop':
                    if self.debounce('reset_estop'):
                        self.request_reset_estop()
                    continue

                if msg_type == 'active':
                    val = data.get('value')
                    if isinstance(val, bool):
                        if self.drive_state == 2 and val:
                            self.get_logger().warn('Currently ESTOP. Send {"type":"reset_estop"} or key "r" first.')
                        else:
                            self.teleop_active = val
                            self.publish_teleop_active()
                    else:
                        self.get_logger().warn(f"Invalid teleop_active value: {val}")
                    continue

                if msg_type == 'cmd':
                    updated = False
                    if "pre_speed_kmh" in data:
                        self.pre_speed_kmh = float(data["pre_speed_kmh"])
                        updated = True
                    if "angle_deg" in data:
                        self.angle_deg = float(data["angle_deg"])
                        updated = True
                    if "pick" in data:
                        self.pick = bool(data["pick"])
                        updated = True
                    if "unload" in data:
                        self.unload = bool(data["unload"])
                        updated = True
                    if "dump" in data:
                        self.dump = bool(data["dump"])
                        updated = True
                    if "pick_action" in data:
                        self.pick_action = bool(data["pick_action"])
                        updated = True

                    if updated:
                        self.publish_teleop_cmd()
                    else:
                        self.get_logger().warn(f"No recognized fields in teleop_cmd: {data}")
                    continue

                self.get_logger().warn(f"Unknown message: {data}")

        except websockets.exceptions.ConnectionClosed:
            self.ws_connected = False
            self.get_logger().warn(f"Client {client_addr} disconnected")
            self.trigger_network_abort(f'websocket disconnected: {client_addr}')
        except Exception as e:
            self.ws_connected = False
            self.get_logger().error(f"Unexpected error: {e}")
            self.trigger_network_abort(f'websocket exception: {e}')

    # ---------------- 本地键盘监听 ----------------
    async def keyboard_loop(self):
        if not self.has_tty or not self.raw_mode:
            return

        while rclpy.ok():
            try:
                dr, _, _ = select.select([self.fd], [], [], 0.05)
                if dr:
                    c = os.read(self.fd, 1).decode(errors='ignore')
                    if c:
                        self.handle_key(c)
            except Exception as e:
                self.get_logger().warn(f'keyboard_loop error: {e}')
                await asyncio.sleep(0.1)

            await asyncio.sleep(0.01)

    # ---------------- ROS spin ----------------
    async def ros_spin_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            await asyncio.sleep(0.01)

    # ---------------- 服务启动 ----------------
    async def start_server(self):
        server = await websockets.serve(
            self.handle_client, "0.0.0.0", 9010,
            ping_interval=20, ping_timeout=10,
        )
        self.get_logger().info("WebSocket teleop bridge listening on ws://0.0.0.0:9010")
        return server


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketTeleopBridge()
    loop = asyncio.get_event_loop()

    async def runner():
        server = await node.start_server()
        tasks = [
            asyncio.create_task(node.ros_spin_loop()),
            asyncio.create_task(node.keyboard_loop()),
        ]
        try:
            await asyncio.gather(*tasks)
        finally:
            server.close()
            await server.wait_closed()

    try:
        loop.run_until_complete(runner())
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down WebSocket teleop bridge...")
    finally:
        node.disable_raw()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
