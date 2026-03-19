#!/usr/bin/env python3
import sys
import os
import termios
import tty
import select
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8
from stack_msgs.msg import StackCommand

HELP = """
teleop_key: keyboard teleop + mode control

全局行驶状态:
  x -> ESTOP (drive_cmd = 2)
  r -> RESET ESTOP / abort
       行为: 先发布零速零角 teleop 命令，再请求 /reset_estop，
            同时切回 RUN + 手动 teleop（但仍保持静止）
  f -> TASK DONE (/task_done True)
  t -> TOGGLE 自动循迹 / 手动 teleop
       注意: t 不再负责退出 ESTOP

  n -> reload .csv 并切到自动循迹

Teleop 速度 (pre_speed_kmh)：
  0 -> 0 km/h
  4/w -> 4 km/h
  8 -> 8 km/h
  1/s -> -4 km/h (后退)

Teleop 方向 (angle_deg)：
  a -> -10 deg (小左)
  d -> +10 deg (小右)
  q -> -20 deg (大左)
  e -> +20 deg (大右)

  c -> 0 deg 0 km/h

作业位:
  j -> TOGGLE dump
  k -> TOGGLE pick
  l -> TOGGLE unload
  h -> TOGGLE pick_action

说明：
- 本节点负责：
  * /drive_cmd
  * /task_done
  * /teleop_active
  * /stack_cmd/teleop
  * /reset_estop
- 如需退出请 Ctrl+C 终止进程。
"""

DEBOUNCE_SEC = 0.15


class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')

        # 发布者
        self.pub_drive_cmd = self.create_publisher(UInt8, '/drive_cmd', 1)
        self.pub_task_done = self.create_publisher(Bool, '/task_done', 1)
        self.pub_teleop_act = self.create_publisher(Bool, '/teleop_active', 1)
        self.pub_teleop_cmd = self.create_publisher(StackCommand, '/stack_cmd/teleop', 1)
        self.pub_restart_path = self.create_publisher(Bool, '/restart_path', 1)
        self.pub_reset_estop = self.create_publisher(Bool, '/reset_estop', 1)

        # 内部状态
        self.drive_state = 1          # 0=PAUSE(不用), 1=RUN, 2=ESTOP
        self.teleop_active = True     # True=手动 teleop, False=自动循迹

        self.pre_speed_kmh = 0.0
        self.angle_deg = 0.0
        self.pick = False
        self.unload = False
        self.dump = False
        self.pick_action = False

        self.last_ts = {}

        self.get_logger().info(HELP)

        # 打开 TTY
        self.fd = None
        self.raw_mode = False
        try:
            self.fd = os.open('/dev/tty', os.O_RDWR | os.O_NOCTTY)
        except Exception:
            self.fd = sys.stdin.fileno()

        self.has_tty = os.isatty(self.fd)
        self.enable_raw()

        # 初始广播一次
        self.publish_drive_cmd()
        self.publish_teleop_active()
        self.publish_teleop_cmd()

    # ---------- TTY raw ----------
    def enable_raw(self):
        if not self.has_tty:
            return False
        try:
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            self.raw_mode = True
            return True
        except Exception as e:
            self.get_logger().warn(f'no TTY raw mode: {e}')
            return False

    def disable_raw(self):
        if self.raw_mode:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            self.raw_mode = False

    def debounce(self, key: str) -> bool:
        now = time.time()
        last = self.last_ts.get(key, 0.0)
        if (now - last) < DEBOUNCE_SEC:
            return False
        self.last_ts[key] = now
        return True

    # ---------- 发布 ----------
    def publish_drive_cmd(self):
        self.pub_drive_cmd.publish(UInt8(data=int(self.drive_state)))
        self.get_logger().info(f'/drive_cmd -> {self.drive_state}')

    def publish_task_done(self):
        self.pub_task_done.publish(Bool(data=True))
        self.get_logger().info('TASK DONE sent')

    def publish_teleop_active(self):
        self.pub_teleop_act.publish(Bool(data=bool(self.teleop_active)))
        self.get_logger().info(
            f'/teleop_active -> {self.teleop_active} '
            f'(True=手动 teleop, False=自动循迹)'
        )

    def publish_reset_estop(self):
        self.pub_reset_estop.publish(Bool(data=True))
        self.get_logger().warn('/reset_estop -> True')

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
            f'TELEOP CMD: speed={self.pre_speed_kmh} km/h, angle={self.angle_deg} deg, '
            f'pick={self.pick}, unload={self.unload}, dump={self.dump}, pick_action={self.pick_action}'
        )

    # ---------- 工具 ----------
    def zero_teleop(self):
        self.pre_speed_kmh = 0.0
        self.angle_deg = 0.0
        self.pick = False
        self.unload = False
        self.dump = False
        self.pick_action = False

    def request_reset_estop(self):
        # 先清零 teleop 命令，避免解锁后窜车
        self.zero_teleop()
        self.publish_teleop_cmd()

        # 解锁后默认进入 RUN + 手动 teleop，但命令为零
        self.teleop_active = True
        self.publish_teleop_active()

        self.drive_state = 1
        self.publish_drive_cmd()

        # 最后发 reset 请求
        self.publish_reset_estop()

    # ---------- 按键 ----------
    def handle_key(self, c: str):
        if not c:
            return
        c = c.lower()

        # restart_path
        if c == 'n':
            if not self.debounce(c):
                return
            self.pub_restart_path.publish(Bool(data=True))
            self.get_logger().info('Request restart path (/restart_path=True)')
            self.drive_state = 1
            self.publish_drive_cmd()
            self.teleop_active = False
            self.publish_teleop_active()
            return

        # ESTOP
        if c == 'x':
            if not self.debounce(c):
                return
            self.drive_state = 2
            self.publish_drive_cmd()
            return

        # RESET ESTOP / abort
        if c == 'r':
            if not self.debounce(c):
                return
            self.request_reset_estop()
            return

        # task done
        if c == 'f':
            if not self.debounce(c):
                return
            self.publish_task_done()
            return

        # 模式切换：只负责自动 / 手动，不负责解锁
        if c == 't':
            if not self.debounce(c):
                return
            if self.drive_state == 2:
                self.get_logger().warn('Currently ESTOP. Use key "r" to reset ESTOP/abort first.')
                return
            self.teleop_active = not self.teleop_active
            self.publish_teleop_active()
            return

        # 速度
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

        # 方向
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

        if c == 'c':
            if not self.debounce(c):
                return
            self.pre_speed_kmh = 0.0
            self.angle_deg = 0.0
            self.publish_teleop_cmd()
            return

        # 作业位
        if c in ('j', 'k', 'l', 'h'):
            if not self.debounce(c):
                return

            if c == 'h':
                self.pick_action = not self.pick_action

                # h 生效时，j/k 全部清零，避免冲突
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

    # ---------- 主循环 ----------
    def spin(self):
        try:
            self.get_logger().info('teleop_key started (Ctrl+C to quit)')
            while rclpy.ok():
                if self.raw_mode:
                    dr, _, _ = select.select([self.fd], [], [], 0.05)
                    if dr:
                        try:
                            c = os.read(self.fd, 1).decode(errors='ignore')
                        except Exception:
                            c = None
                        if c:
                            self.handle_key(c)
                else:
                    line = sys.stdin.readline()
                    if line:
                        c = line.strip().lower()[:1]
                        if c:
                            self.handle_key(c)

                rclpy.spin_once(self, timeout_sec=0.0)

        finally:
            self.disable_raw()
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()
    node.spin()


if __name__ == '__main__':
    main()
