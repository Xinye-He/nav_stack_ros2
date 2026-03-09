#!/usr/bin/env python3
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import UInt8, Bool
from stack_msgs.msg import StackCommand, StackCanStatus
from builtin_interfaces.msg import Time

try:
    import can
    HAVE_CAN = True
except Exception:
    HAVE_CAN = False


class StackCanExecutor(Node):
    DS_PAUSED = 0
    DS_RUNNING = 1
    DS_ESTOP  = 2

    def __init__(self):
        super().__init__('stack_can_executor')

        # CAN 参数
        self.declare_parameter('enable_can', True)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_extended', True)
        self.declare_parameter('can_id_status', 0x18FED188)

        # 速度编码参数
        self.declare_parameter('vcu_speed_raw_offset_kmh', -50.0)
        self.declare_parameter('vcu_speed_raw_res_kmh_per_lsb', 0.5)

        # 读参数
        self.enable_can = bool(self.get_parameter('enable_can').value)
        self.can_interface = self.get_parameter('can_interface').value
        self.can_extended = bool(self.get_parameter('can_extended').value)
        _id_val = self.get_parameter('can_id_status').value
        try:
            if isinstance(_id_val, (int, float)):
                self.can_id_status = int(_id_val)
            else:
                self.can_id_status = int(str(_id_val), 0)
        except Exception:
            self.can_id_status = 0x18FED188

        self.vcu_speed_raw_offset_kmh = float(self.get_parameter('vcu_speed_raw_offset_kmh').value)
        self.vcu_speed_raw_res_kmh_per_lsb = float(self.get_parameter('vcu_speed_raw_res_kmh_per_lsb').value)

        # CAN 总线
        self.can_bus = None
        if self.enable_can:
            if not HAVE_CAN:
                self.get_logger().error("python-can not installed, CAN disabled")
            else:
                try:
                    self.can_bus = can.Bus(channel=self.can_interface, bustype='socketcan', fd=False)
                    self.get_logger().info(f"CAN enabled on {self.can_interface}, status_id=0x{self.can_id_status:X}")
                except Exception as e:
                    self.get_logger().error(f"Open CAN failed: {e}")
                    self.can_bus = None

        # 最新命令缓存
        self.cmd_traj   = None  # StackCommand
        self.cmd_teleop = None
        self.cmd_bale   = None

        # 状态
        self.drive_state   = self.DS_PAUSED
        self.remote_req    = False  # 实体遥控
        self.teleop_active = False  # 上位机接管
        self.bale_active   = False  # 草捆对准接管

        # 订阅命令源
        qos = QoSProfile(depth=1)
        self.create_subscription(StackCommand, '/stack_cmd/traj',
                                 self._on_traj_cmd, qos)
        self.create_subscription(StackCommand, '/stack_cmd/teleop',
                                 self._on_teleop_cmd, qos)
        self.create_subscription(StackCommand, '/stack_cmd/bale',
                                 self._on_bale_cmd, qos)

        # 订阅状态
        self.create_subscription(UInt8, '/drive_cmd',
                                 lambda m: setattr(self, 'drive_state', int(m.data)), 1)
        self.create_subscription(Bool, '/remote_req',
                                 lambda m: setattr(self, 'remote_req', bool(m.data)), 1)
        self.create_subscription(Bool, '/teleop_active',
                                 lambda m: setattr(self, 'teleop_active', bool(m.data)), 1)
        self.create_subscription(Bool, '/bale_active',
                                 lambda m: setattr(self, 'bale_active', bool(m.data)), 1)

        # 定时器：20Hz 输出 CAN
        self.create_timer(0.05, self.loop)
        self.pub_can_status = self.create_publisher(StackCanStatus, '/stack_can/status', 10)

        self.get_logger().info("StackCanExecutor initialized")

    def _on_traj_cmd(self, msg: StackCommand):
        self.cmd_traj = msg

    def _on_teleop_cmd(self, msg: StackCommand):
        self.cmd_teleop = msg

    def _on_bale_cmd(self, msg: StackCommand):
        self.cmd_bale = msg

    def loop(self):
        # 没有 CAN 或总线不可用
        if not (self.enable_can and self.can_bus):
            return

        # 1) 全局 ESTOP 优先
        if self.drive_state == self.DS_ESTOP:
            self.send_can(speed_kmh=0.0,
                          angle_deg=0.0,
                          dist_m=0.0,
                          pick=False, unload=False, dump=False, pick_action=False,
                          remote_bit=True,
                          estop=True,
                          drive=False)
            return

        # 2) 实体遥控 remote_req
        if self.remote_req:
            # remote_bit=1，其余全停，让 VCU 切到实体遥控
            self.send_can(speed_kmh=0.0,
                          angle_deg=0.0,
                          dist_m=0.0,
                          pick=False, unload=False, dump=False, pick_action=False,
                          remote_bit=True,
                          estop=False,
                          drive=False)
            return

        # 3) 选取当前控制源（teleop > bale > traj）
        cmd = None
        if self.teleop_active and self.cmd_teleop is not None and self.cmd_teleop.valid:
            cmd = self.cmd_teleop
        elif self.bale_active and self.cmd_bale is not None and self.cmd_bale.valid:
            cmd = self.cmd_bale
        elif self.cmd_traj is not None and self.cmd_traj.valid:
            cmd = self.cmd_traj

        if cmd is None:
            # 没有任何有效命令，安全起见停住
            self.send_can(0.0, 0.0, 0.0, False, False, False, False,
                          remote_bit=False, estop=False, drive=False)
            return

        speed_kmh = float(cmd.pre_speed_kmh)
        angle_deg = float(cmd.angle_deg)

        # 4) 发送 CAN
        self.send_can(speed_kmh=speed_kmh,
                      angle_deg=angle_deg,
                      dist_m=float(cmd.dist_to_target_m),
                      pick=bool(cmd.pick),
                      unload=bool(cmd.unload),
                      dump=bool(cmd.dump),
                      pick_action=bool(cmd.pick_action),
                      remote_bit=False,
                      estop=False,
                      drive=(self.drive_state == self.DS_RUNNING))

    def send_can(self, speed_kmh: float,
                 angle_deg: float,
                 dist_m: float,
                 pick: bool,
                 unload: bool,
                 dump: bool,
                 pick_action: bool,
                 remote_bit: bool,
                 estop: bool,
                 drive: bool):

        # 距离编码：0.2 m / LSB, 最大约30m
        dist_raw = int(round(max(0.0, dist_m) / 0.2))
        dist_raw = max(0, min(150, dist_raw))

        # 方向角编码：直接用相对转向角（正=右转，负=左转）
        steer_deg = float(angle_deg)
        if steer_deg > 180.0:
            steer_deg = 180.0
        if steer_deg < -180.0:
            steer_deg = -180.0
        angle_raw = int(round(steer_deg + 180.0))
        angle_raw = max(0, min(359, angle_raw))

        # 速度编码：km/h -> 原始字节
        try:
            raw = (float(speed_kmh) - self.vcu_speed_raw_offset_kmh) / self.vcu_speed_raw_res_kmh_per_lsb
            speed_raw = int(round(raw))
        except Exception:
            speed_raw = 0
        speed_raw = max(0, min(255, speed_raw))

        # 第5字节位定义：
        # bit0: pick
        # bit2: unload
        # bit4: remote_bit
        # bit5: dump
        # bit6: pick_action
        byte5 = 0
        if pick:
            byte5 |= 0x01
        if unload:
            byte5 |= 0x01 << 2
        if remote_bit:
            byte5 |= 0x01 << 4
        if dump:
            byte5 |= 0x01 << 5
        if pick_action:
            byte5 |= 0x01 << 6

        # 第6字节：bit0 estop, bit1 drive
        byte6 = 0
        if estop:
            byte6 |= 0x01
        if drive:
            byte6 |= 0x01 << 1

        payload = bytearray(8)
        struct.pack_into('<H', payload, 0, dist_raw)
        struct.pack_into('<H', payload, 2, angle_raw)
        payload[4] = speed_raw
        payload[5] = byte5
        payload[6] = byte6
        payload[7] = 0x00

        # --- 发布 StackCanStatus 到 /stack_can/status ---
        status = StackCanStatus()
        status.stamp = self.get_clock().now().to_msg()
        status.id = int(self.can_id_status)
        status.is_extended = bool(self.can_extended)

        # 原始 payload
        status.data = list(payload)  # uint8[8]

        # 解码字段（供复盘/可视化）
        status.dist_m = float(dist_raw) * 0.2

        # 方向角：从angle_raw还原，0~359 -> -180~+180，再以右正左负
        steer_deg = float(angle_raw) - 180.0
        if steer_deg > 180.0:
            steer_deg -= 360.0
        elif steer_deg < -180.0:
            steer_deg += 360.0
        status.steer_deg = steer_deg

        # 速度：用同样的 offset/res 反算
        status.speed_kmh = float(speed_raw) * self.vcu_speed_raw_res_kmh_per_lsb + self.vcu_speed_raw_offset_kmh

        # 各位标志，和 send_can() 里计算 byte5/byte6 时保持一致
        status.pick        = bool(byte5 & 0x01)
        status.unload      = bool(byte5 & (0x01 << 2))
        status.remote      = bool(byte5 & (0x01 << 4))
        status.dump        = bool(byte5 & (0x01 << 5))
        status.pick_action = bool(byte5 & (0x01 << 6))

        status.estop       = bool(byte6 & 0x01)
        status.drive       = bool(byte6 & (0x01 << 1))

        self.pub_can_status.publish(status)

        try:
            msg = can.Message(arbitration_id=self.can_id_status,
                              data=payload,
                              is_extended_id=self.can_extended)
            self.can_bus.send(msg, timeout=0.001)
        except Exception:
            # 发送失败这边不抛异常，避免频繁崩溃
            pass


def main(args=None):
    rclpy.init(args=args)
    node = StackCanExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
