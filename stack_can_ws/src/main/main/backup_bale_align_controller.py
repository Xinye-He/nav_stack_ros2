#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import Bool, UInt8
from stack_msgs.msg import StackCommand, BaleTarget


class BaleAlignController(Node):
    """
    草捆对准控制块：
      - 在任务点等待状态下 (/at_task_waiting=True)，根据 /bale_target 的角度先原地旋转对准草捆，
      - 对准后自动发送一次 pick_action=True 触发 VCU 捡拾动作，
      - 等待一段时间后结束对准，发 /task_done 通知轨迹继续。
    输出通过 /stack_cmd/bale + /bale_active 接入 stack_can_executor。
    """

    DS_PAUSED = 0
    DS_RUNNING = 1
    DS_ESTOP = 2

    def __init__(self):
        super().__init__('bale_align_controller')

        # 参数（可放到 params.yaml 调）
        self.declare_parameter('angle_align_deg', 3.0)        # 认为对准的角度阈值
        self.declare_parameter('angle_large_deg', 50.0)       # 大角/小角切换阈值
        self.declare_parameter('spin_large_cmd_deg', 20.0)    # 大角旋转指令
        self.declare_parameter('spin_small_cmd_deg', 10.0)    # 小角旋转指令
        self.declare_parameter('pick_action_hold_s', 40.0)     # pick_action 后等待时间
        self.declare_parameter('min_dist_m', 0.0)             # 太近/太远不对准的距离下限
        self.declare_parameter('max_dist_m', 15.0)            # 距离上限
        # 先靠近逻辑用
        self.declare_parameter('approach_dist_m', 2.0)        # 小于这个距离才允许捡拾
        self.declare_parameter('approach_speed_kmh', 4.0)     # 靠近时低速（km/h）
        self.declare_parameter('too_close_dist_m', 1.0)       # 小于这个距离认为“太近需要后退”
        self.declare_parameter('retreat_speed_kmh', 4.0)      # 后退速度（km/h，命令里会取负号）

        # 对准后“停稳再捡”的参数
        self.declare_parameter('pick_settle_angle_deg', 1.0)   # 认为已经稳住的更严格角度阈值
        self.declare_parameter('pick_settle_time_s', 1.0)      # 满足稳态角度后，继续等待的时间

        self.pick_settle_angle_deg = float(self.get_parameter('pick_settle_angle_deg').value)
        self.pick_settle_time_s = float(self.get_parameter('pick_settle_time_s').value)

        # 内部状态：对准稳态开始时间
        self.align_stable_since: Optional[float] = None

        self.angle_align_deg = float(self.get_parameter('angle_align_deg').value)
        self.angle_large_deg = float(self.get_parameter('angle_large_deg').value)
        self.spin_large_cmd_deg = float(self.get_parameter('spin_large_cmd_deg').value)
        self.spin_small_cmd_deg = float(self.get_parameter('spin_small_cmd_deg').value)
        self.pick_action_hold_s = float(self.get_parameter('pick_action_hold_s').value)
        self.min_dist_m = float(self.get_parameter('min_dist_m').value)
        self.max_dist_m = float(self.get_parameter('max_dist_m').value)

        self.approach_dist_m = float(self.get_parameter('approach_dist_m').value)
        self.approach_speed_kmh = float(self.get_parameter('approach_speed_kmh').value)
        self.too_close_dist_m = float(self.get_parameter('too_close_dist_m').value)
        self.retreat_speed_kmh = float(self.get_parameter('retreat_speed_kmh').value)

        # 订阅
        self.at_task_waiting = False
        self.drive_state = self.DS_PAUSED
        self.current_target: Optional[BaleTarget] = None

        self.create_subscription(BaleTarget, '/bale_target',
                                 self.on_bale_target, 10)
        self.create_subscription(Bool, '/at_task_waiting',
                                 self.on_task_waiting, 10)
        self.create_subscription(UInt8, '/drive_cmd',
                                 self.on_drive_cmd, 10)

        # 发布
        self.pub_bale_cmd = self.create_publisher(StackCommand, '/stack_cmd/bale', 1)
        qos_latched = QoSProfile(depth=1)
        qos_latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.pub_bale_active = self.create_publisher(Bool, '/bale_active', qos_latched)
        self.pub_task_done = self.create_publisher(Bool, '/task_done', 1)

        # 内部状态
        self.active = False                 # 是否当前在控制
        self.pick_action_sent_time = None   # 触发 pick_action 的时间戳（秒）
        self.retreating = False             # 是否正在执行“后退到捡拾距离”

        # 定时器
        self.create_timer(0.10, self.loop)  # 20Hz

        self.get_logger().info("BaleAlignController initialized")

    # ---- 回调 ----
    def on_bale_target(self, msg: BaleTarget):
        if msg.valid:
            self.current_target = msg
        else:
            self.current_target = None

    def on_task_waiting(self, msg: Bool):
        self.at_task_waiting = bool(msg.data)

    def on_drive_cmd(self, msg: UInt8):
        self.drive_state = int(msg.data)

    # ---- 状态切换 ----
    def set_active(self, flag: bool):
        if self.active != flag:
            self.active = flag
            self.pub_bale_active.publish(Bool(data=flag))
            self.get_logger().info(f"bale_active -> {flag}")
            if not flag:
                # 退出控制时清除 pick_action 相关状态
                self.pick_action_sent_time = None
                self.align_stable_since = None
                self.retreating = False

    def loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # ------------------------------------------------------------------
        # 情况 1：已经触发过 pick_action（捡拾等待阶段）
        # ------------------------------------------------------------------
        if self.pick_action_sent_time is not None:
            # 急停时仍然可以中断整个流程
            if self.drive_state == self.DS_ESTOP:
                self.set_active(False)
                self.get_logger().warn("ESTOP during pick_action, abort sequence")
                return

            # 锁定 active，不再受目标/角度等影响
            self.set_active(True)

            # 距离只是附带信息
            if self.current_target is not None:
                dist = float(self.current_target.distance_m)
            else:
                dist = 0.0

            dt = now - self.pick_action_sent_time

            # 等待阶段：持续发 pick_action=True 的静止命令
            cmd = StackCommand()
            cmd.pre_speed_kmh = 0.0
            cmd.angle_deg = 0.0
            cmd.dist_to_target_m = dist
            cmd.pick = False
            cmd.unload = False
            cmd.dump = False
            cmd.pick_action = True
            cmd.valid = True
            self.pub_bale_cmd.publish(cmd)

            # 时间到了：拉低 pick_action，并结束本次捡拾流程
            if dt >= self.pick_action_hold_s:
                cmd_done = StackCommand()
                cmd_done.pre_speed_kmh = 0.0
                cmd_done.angle_deg = 0.0
                cmd_done.dist_to_target_m = dist
                cmd_done.pick = False
                cmd_done.unload = False
                cmd_done.dump = False
                cmd_done.pick_action = False
                cmd_done.valid = True
                self.pub_bale_cmd.publish(cmd_done)

                self.set_active(False)
                self.pub_task_done.publish(Bool(data=True))
                self.get_logger().info(
                    f"Bale pick_action done, dt={dt:.1f}s, publish /task_done"
                )
                # set_active(False) 会把 pick_action_sent_time 置回 None
            return

        # ------------------------------------------------------------------
        # 情况 2：尚未触发 pick_action —— 对准 + 靠近 + 过近后退
        # ------------------------------------------------------------------

        # 0) 基本前置条件：需要 RUN 状态 + at_task_waiting=True
        if self.drive_state != self.DS_RUNNING or not self.at_task_waiting:
            self.set_active(False)
            return

        # 1) 必须有有效目标
        tgt = self.current_target
        if tgt is None:
            self.set_active(False)
            return

        dist = float(tgt.distance_m)
        angle = float(tgt.angle_deg)  # 右正左负

        # 2) 距离太远（超出关注上限），不控制
        if dist > self.max_dist_m:
            self.set_active(False)
            return
        
        # 只要角度还比较大，或者距离还没进入捡拾距离，就认为还没进入“稳态对准”阶段
        if abs(angle) > self.angle_align_deg or dist > self.approach_dist_m:
            self.align_stable_since = None

        # ------------------------------------------------------------------
        # 2.5）过近后退逻辑：
        #  - 一旦发现 dist < too_close_dist_m（默认1m），就开始后退，
        #  - 一直退到 dist >= approach_dist_m（默认2m），期间持续发 -retreat_speed_kmh
        # ------------------------------------------------------------------
        if self.retreating or dist < self.too_close_dist_m:
            # 如果已经退到足够远（>= approach_dist_m），结束后退，继续后面的对准/靠近逻辑
            if dist >= self.approach_dist_m:
                self.retreating = False
            else:
                # 还在过近区间内：保持后退
                self.retreating = True
                self.set_active(True)

                cmd = StackCommand()
                cmd.pre_speed_kmh = -self.retreat_speed_kmh   # 负号表示后退
                cmd.angle_deg = 0.0                           # 直线后退
                cmd.dist_to_target_m = dist
                cmd.pick = False
                cmd.unload = False
                cmd.dump = False
                cmd.pick_action = False
                cmd.valid = True
                self.pub_bale_cmd.publish(cmd)

                # 可选日志
                # self.get_logger().debug(
                #     f"Retreating from bale: dist={dist:.2f}m < {self.approach_dist_m}m, speed={cmd.pre_speed_kmh} km/h"
                # )
                return

        # ------------------------------------------------------------------
        # 到这里：距离在 [too_close_dist_m, max_dist_m]，且不处在后退阶段
        # 接下来执行：先对准 → 再靠近 → 触发捡拾
        # ------------------------------------------------------------------
        self.set_active(True)

        # 阶段 A：对准优先 —— 任意时刻只要偏角 > angle_align_deg，就先原地对准
        if abs(angle) > self.angle_align_deg:
            self.pick_action_sent_time = None  # 还没到捡拾阶段

            cmd = StackCommand()
            cmd.pre_speed_kmh = 0.0  # 原地旋转

            # 大角/小角两档
            if abs(angle) >= self.angle_large_deg:
                cmd.angle_deg = self.spin_large_cmd_deg if angle > 0.0 else -self.spin_large_cmd_deg
            else:
                cmd.angle_deg = self.spin_small_cmd_deg if angle > 0.0 else -self.spin_small_cmd_deg

            cmd.dist_to_target_m = dist
            cmd.pick = False
            cmd.unload = False
            cmd.dump = False
            cmd.pick_action = False
            cmd.valid = True

            self.pub_bale_cmd.publish(cmd)
            self.get_logger().debug(
                f"Aligning bale: angle={angle:.1f}deg -> cmd.angle={cmd.angle_deg:.1f}deg, dist={dist:.2f}m"
            )
            return

        # 现在：角度已经在对准阈值内（abs(angle) <= angle_align_deg）

        # 阶段 B：靠近 —— 对准后，如果距离 > approach_dist_m，就低速直线靠近
        if dist > self.approach_dist_m:
            cmd = StackCommand()
            cmd.pre_speed_kmh = self.approach_speed_kmh  # 比如 4 km/h
            cmd.angle_deg = 0.0                          # 已经对准，直线靠近
            cmd.dist_to_target_m = dist
            cmd.pick = False
            cmd.unload = False
            cmd.dump = False
            cmd.pick_action = False
            cmd.valid = True

            self.pub_bale_cmd.publish(cmd)
            # self.get_logger().debug(
            #     f"Approaching bale: dist={dist:.2f}m > {self.approach_dist_m}m, speed={cmd.pre_speed_kmh} km/h"
            # )
            return

        # 阶段 C：已经在捡拾距离以内（dist <= approach_dist_m 且 abs(angle) <= angle_align_deg）
        # 这里分两步：
        #   1）先要求角度进入更严格的稳态范围 pick_settle_angle_deg，
        #   2）在该范围内保持 pick_settle_time_s 以上，才真正触发捡拾。

        # 1）角度还在轻微晃动：先发停车命令但不触发捡拾，等待自然停稳
        if abs(angle) > self.pick_settle_angle_deg:
            self.align_stable_since = None

            cmd = StackCommand()
            cmd.pre_speed_kmh = 0.0      # 停车等待
            cmd.angle_deg = 0.0
            cmd.dist_to_target_m = dist
            cmd.pick = False
            cmd.unload = False
            cmd.dump = False
            cmd.pick_action = False
            cmd.valid = True

            self.pub_bale_cmd.publish(cmd)
            # self.get_logger().debug(
            #     f"Waiting for angle to settle: dist={dist:.2f}m, angle={angle:.2f}deg"
            # )
            return

        # 2）角度已经足够小：开始计时，要求连续稳定一段时间
        if self.align_stable_since is None:
            self.align_stable_since = now

        if (now - self.align_stable_since) < self.pick_settle_time_s:
            # 还在稳定计时窗口内：继续保持停车，但先不捡
            cmd = StackCommand()
            cmd.pre_speed_kmh = 0.0
            cmd.angle_deg = 0.0
            cmd.dist_to_target_m = dist
            cmd.pick = False
            cmd.unload = False
            cmd.dump = False
            cmd.pick_action = False
            cmd.valid = True

            self.pub_bale_cmd.publish(cmd)
            # self.get_logger().debug(
            #     f"Holding before pick: stable_for={now - self.align_stable_since:.2f}s"
            # )
            return

        # 3）稳态时间达到要求：真正触发捡拾
        cmd = StackCommand()
        cmd.pre_speed_kmh = 0.0      # 捡拾时原地不动
        cmd.angle_deg = 0.0
        cmd.dist_to_target_m = dist
        cmd.pick = False
        cmd.unload = False
        cmd.dump = False
        cmd.pick_action = True
        cmd.valid = True

        self.pub_bale_cmd.publish(cmd)
        self.pick_action_sent_time = now
        self.get_logger().info(
            f"Bale aligned & settled (dist={dist:.2f}m, |angle|={abs(angle):.1f}deg), "
            f"stable_for={now - self.align_stable_since:.2f}s, send pick_action=True"
        )
"""        
        # 阶段 C：满足“接近 + 对准”条件 —— 触发捡拾
        # 到这里：dist <= approach_dist_m 且 abs(angle) <= angle_align_deg
        cmd = StackCommand()
        cmd.pre_speed_kmh = 0.0      # 捡拾时原地不动
        cmd.angle_deg = 0.0
        cmd.dist_to_target_m = dist
        cmd.pick = False
        cmd.unload = False
        cmd.dump = False
        cmd.pick_action = True
        cmd.valid = True

        self.pub_bale_cmd.publish(cmd)
        self.pick_action_sent_time = now
        self.get_logger().info(
            f"Bale aligned & at pick distance (dist={dist:.2f}m, |angle|={abs(angle):.1f}deg), send pick_action=True"
        )
"""

def main(args=None):
    rclpy.init(args=args)
    node = BaleAlignController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
