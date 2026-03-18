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
    草捆末端对准控制器（周期动作版）

    核心思路：
    1. 不再做高频逐帧反应式“抖动控制”
    2. 原地对准采用“固定转向周期”：
       - 左/右固定转向指令
       - 连续 pulse_frames 帧
       - 再 pause_frames 帧中性命令
       - 一个完整周期结束后，再重新观察目标角度
    3. 适配已经实测出的底盘特性：
       period=0.10s, pulse=5, pause=1, 一个周期约转 2.6°
    4. 对准后再低速靠近；偏差重新变大，则回到原地周期修正
    5. 靠近到阈值后，拉高 pick_action 一段时间，然后发 /task_done
    """

    DS_PAUSED = 0
    DS_RUNNING = 1
    DS_ESTOP = 2

    def __init__(self):
        super().__init__('bale_align_controller')

        # --------------------------------------------------
        # 参数
        # --------------------------------------------------
        # 角度滤波
        self.declare_parameter('angle_lpf_alpha', 0.25)

        # 对准判定（双阈值滞回）
        self.declare_parameter('align_enter_deg', 2.0)
        self.declare_parameter('align_exit_deg', 6.0)

        # 固定转向周期参数（基于你的实测）
        self.declare_parameter('turn_cmd_angle_deg', 10.0)   # 固定档位角，实际发送时左负右正
        self.declare_parameter('turn_frame_period_s', 0.10)  # 每帧周期
        self.declare_parameter('turn_pulse_frames', 5)       # 连发转向帧数
        self.declare_parameter('turn_pause_frames', 1)       # 中性帧数
        self.declare_parameter('turn_cycle_yaw_deg', 2.6)    # 记录用途，当前主要用于日志

        # 靠近阶段
        self.declare_parameter('approach_dist_m', 2.0)
        self.declare_parameter('approach_speed_kmh', 4.0)

        # 过近后退
        self.declare_parameter('too_close_dist_m', 1.0)
        self.declare_parameter('retreat_speed_kmh', 4.0)

        # 目标有效范围
        self.declare_parameter('min_dist_m', 0.0)
        self.declare_parameter('max_dist_m', 15.0)

        # 目标超时
        self.declare_parameter('target_timeout_s', 0.50)

        # pick_action 保持时长
        self.declare_parameter('pick_action_hold_s', 30.0)

        # --------------------------------------------------
        # 取参数
        # --------------------------------------------------
        self.angle_lpf_alpha = float(self.get_parameter('angle_lpf_alpha').value)

        self.align_enter_deg = float(self.get_parameter('align_enter_deg').value)
        self.align_exit_deg = float(self.get_parameter('align_exit_deg').value)

        self.turn_cmd_angle_deg = float(self.get_parameter('turn_cmd_angle_deg').value)
        self.turn_frame_period_s = float(self.get_parameter('turn_frame_period_s').value)
        self.turn_pulse_frames = int(self.get_parameter('turn_pulse_frames').value)
        self.turn_pause_frames = int(self.get_parameter('turn_pause_frames').value)
        self.turn_cycle_yaw_deg = float(self.get_parameter('turn_cycle_yaw_deg').value)

        self.approach_dist_m = float(self.get_parameter('approach_dist_m').value)
        self.approach_speed_kmh = float(self.get_parameter('approach_speed_kmh').value)

        self.too_close_dist_m = float(self.get_parameter('too_close_dist_m').value)
        self.retreat_speed_kmh = float(self.get_parameter('retreat_speed_kmh').value)

        self.min_dist_m = float(self.get_parameter('min_dist_m').value)
        self.max_dist_m = float(self.get_parameter('max_dist_m').value)

        self.target_timeout_s = float(self.get_parameter('target_timeout_s').value)
        self.pick_action_hold_s = float(self.get_parameter('pick_action_hold_s').value)

        # --------------------------------------------------
        # 订阅
        # --------------------------------------------------
        self.at_task_waiting = False
        self.drive_state = self.DS_PAUSED
        self.current_target: Optional[BaleTarget] = None
        self.last_target_recv_time = None

        self.create_subscription(BaleTarget, '/bale_target', self.on_bale_target, 10)
        self.create_subscription(Bool, '/at_task_waiting', self.on_task_waiting, 10)
        self.create_subscription(UInt8, '/drive_cmd', self.on_drive_cmd, 1)

        # --------------------------------------------------
        # 发布
        # --------------------------------------------------
        self.pub_bale_cmd = self.create_publisher(StackCommand, '/stack_cmd/bale', 1)

        qos_latched = QoSProfile(depth=1)
        qos_latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub_bale_active = self.create_publisher(Bool, '/bale_active', qos_latched)

        self.pub_task_done = self.create_publisher(Bool, '/task_done', 1)

        # --------------------------------------------------
        # 内部状态
        # --------------------------------------------------
        self.active = False
        self.pick_action_sent_time = None
        self.retreating = False

        # 角度滤波
        self.filtered_angle_deg: Optional[float] = None

        # 滞回对准状态
        self.aligned_state = False

        # 执行子状态机
        # IDLE / TURN_LEFT / TURN_RIGHT / APPROACH
        self.exec_mode = 'IDLE'
        # NONE / PULSE / PAUSE
        self.turn_phase = 'NONE'
        self.turn_frames_left = 0

        # 定时器：直接按你实测有效的节拍跑
        self.create_timer(self.turn_frame_period_s, self.loop)

        self.get_logger().info('BaleAlignController (cycle-based version) initialized')

    # --------------------------------------------------
    # 回调
    # --------------------------------------------------
    def on_bale_target(self, msg: BaleTarget):
        now = self.now_s()

        if not msg.valid:
            self.current_target = None
            self.last_target_recv_time = now
            return

        self.current_target = msg
        self.last_target_recv_time = now

        raw_angle = float(msg.angle_deg)

        if self.filtered_angle_deg is None:
            self.filtered_angle_deg = raw_angle
        else:
            a = self.angle_lpf_alpha
            self.filtered_angle_deg = a * raw_angle + (1.0 - a) * self.filtered_angle_deg

    def on_task_waiting(self, msg: Bool):
        self.at_task_waiting = bool(msg.data)

    def on_drive_cmd(self, msg: UInt8):
        self.drive_state = int(msg.data)

    # --------------------------------------------------
    # 工具函数
    # --------------------------------------------------
    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def target_is_fresh(self, now: float) -> bool:
        if self.current_target is None:
            return False
        if self.last_target_recv_time is None:
            return False
        return (now - self.last_target_recv_time) <= self.target_timeout_s

    def reset_control_state(self):
        self.retreating = False
        self.aligned_state = False
        self.filtered_angle_deg = None

        self.exec_mode = 'IDLE'
        self.turn_phase = 'NONE'
        self.turn_frames_left = 0

    def set_active(self, flag: bool):
        if self.active != flag:
            self.active = flag
            self.pub_bale_active.publish(Bool(data=flag))
            self.get_logger().info(f'bale_active -> {flag}')

        if not flag:
            self.pick_action_sent_time = None
            self.reset_control_state()

    def make_cmd(
        self,
        speed_kmh: float,
        angle_deg: float,
        dist_m: float,
        pick_action: bool = False,
        valid: bool = True,
    ) -> StackCommand:
        cmd = StackCommand()
        cmd.pre_speed_kmh = float(speed_kmh)
        cmd.angle_deg = float(angle_deg)
        cmd.dist_to_target_m = float(dist_m)
        cmd.pick = False
        cmd.unload = False
        cmd.dump = False
        cmd.pick_action = bool(pick_action)
        cmd.valid = bool(valid)
        return cmd

    def update_aligned_state(self, angle_abs: float):
        """
        双阈值滞回
        - 未对准 -> abs(angle) <= enter 时进入已对准
        - 已对准 -> abs(angle) >= exit 时退出已对准
        """
        if not self.aligned_state:
            if angle_abs <= self.align_enter_deg:
                self.aligned_state = True
        else:
            if angle_abs >= self.align_exit_deg:
                self.aligned_state = False

    # --------------------------------------------------
    # 固定转向周期
    # --------------------------------------------------
    def start_turn_cycle(self, direction: str):
        """
        direction: 'LEFT' or 'RIGHT'
        """
        if direction not in ('LEFT', 'RIGHT'):
            return

        self.exec_mode = f'TURN_{direction}'
        self.turn_phase = 'PULSE'
        self.turn_frames_left = self.turn_pulse_frames

        sign = -1 if direction == 'LEFT' else 1
        est_yaw = sign * self.turn_cycle_yaw_deg

        self.get_logger().info(
            f'start turn cycle: {direction}, '
            f'cmd={sign*self.turn_cmd_angle_deg:.1f} deg, '
            f'frames={self.turn_pulse_frames}+{self.turn_pause_frames}, '
            f'est_yaw={est_yaw:+.2f} deg/cycle'
        )

    def step_turn_cycle(self, dist: float):
        """
        执行一个未完成的固定转向周期。
        返回 True 表示本轮已经处理，不要进入其他逻辑。
        """
        if self.exec_mode == 'TURN_LEFT':
            turn_angle = -self.turn_cmd_angle_deg
        elif self.exec_mode == 'TURN_RIGHT':
            turn_angle = +self.turn_cmd_angle_deg
        else:
            return False

        if self.turn_phase == 'PULSE':
            cmd = self.make_cmd(
                speed_kmh=0.0,
                angle_deg=turn_angle,
                dist_m=dist,
                pick_action=False,
                valid=True,
            )
            self.pub_bale_cmd.publish(cmd)
            self.turn_frames_left -= 1

            if self.turn_frames_left <= 0:
                self.turn_phase = 'PAUSE'
                self.turn_frames_left = self.turn_pause_frames

            return True

        if self.turn_phase == 'PAUSE':
            cmd = self.make_cmd(
                speed_kmh=0.0,
                angle_deg=0.0,
                dist_m=dist,
                pick_action=False,
                valid=True,
            )
            self.pub_bale_cmd.publish(cmd)
            self.turn_frames_left -= 1

            if self.turn_frames_left <= 0:
                self.exec_mode = 'IDLE'
                self.turn_phase = 'NONE'

            return True

        return False

    # --------------------------------------------------
    # 主循环
    # --------------------------------------------------
    def loop(self):
        now = self.now_s()

        # --------------------------------------
        # 1) 已进入 pick_action 保持阶段
        # --------------------------------------
        if self.pick_action_sent_time is not None:
            if self.drive_state == self.DS_ESTOP:
                self.get_logger().warn('ESTOP during pick_action, abort bale sequence')
                self.set_active(False)
                return

            self.set_active(True)

            dist = 0.0
            if self.current_target is not None:
                dist = float(self.current_target.distance_m)

            dt = now - self.pick_action_sent_time

            cmd_hold = self.make_cmd(
                speed_kmh=0.0,
                angle_deg=0.0,
                dist_m=dist,
                pick_action=True,
                valid=True,
            )
            self.pub_bale_cmd.publish(cmd_hold)

            if dt >= self.pick_action_hold_s:
                cmd_done = self.make_cmd(
                    speed_kmh=0.0,
                    angle_deg=0.0,
                    dist_m=dist,
                    pick_action=False,
                    valid=True,
                )
                self.pub_bale_cmd.publish(cmd_done)
                self.set_active(False)
                self.pub_task_done.publish(Bool(data=True))
                self.get_logger().info(
                    f'pick_action hold done: {dt:.1f}s, publish /task_done'
                )
            return

        # --------------------------------------
        # 2) 前置条件判断
        # --------------------------------------
        if self.drive_state != self.DS_RUNNING or not self.at_task_waiting:
            self.set_active(False)
            return

        if not self.target_is_fresh(now):
            self.set_active(False)
            return

        tgt = self.current_target
        if tgt is None:
            self.set_active(False)
            return

        if self.filtered_angle_deg is None:
            self.set_active(False)
            return

        dist = float(tgt.distance_m)
        angle = float(self.filtered_angle_deg)
        angle_abs = abs(angle)

        if dist < self.min_dist_m or dist > self.max_dist_m:
            self.set_active(False)
            return

        self.set_active(True)

        # 滞回更新
        self.update_aligned_state(angle_abs)

        # --------------------------------------
        # 3) 如果当前还在执行一个固定转向周期
        #    就先执行完，不要中途改主意
        # --------------------------------------
        if self.exec_mode in ('TURN_LEFT', 'TURN_RIGHT'):
            self.step_turn_cycle(dist)
            return

        # --------------------------------------
        # 4) 过近：先后退
        # --------------------------------------
        if self.retreating or dist < self.too_close_dist_m:
            if dist >= self.approach_dist_m:
                self.retreating = False
            else:
                self.retreating = True
                cmd = self.make_cmd(
                    speed_kmh=-self.retreat_speed_kmh,
                    angle_deg=0.0,
                    dist_m=dist,
                    pick_action=False,
                    valid=True,
                )
                self.pub_bale_cmd.publish(cmd)
                return

        # --------------------------------------
        # 5) 未对准：按固定周期做一步原地修正
        #    注意：
        #    BaleTarget.angle_deg 的正负语义要和你当前检测节点保持一致
        #
        #    这里沿用之前工程里的约定：
        #      angle < 0  ==> 目标在左，需要左转
        #      angle > 0  ==> 目标在右，需要右转
        #
        #    如果你现场发现方向反了，只需要把 LEFT/RIGHT 对调即可
        # --------------------------------------
        if not self.aligned_state:
            if angle < 0.0:
                self.start_turn_cycle('LEFT')
                self.step_turn_cycle(dist)
                return

            if angle > 0.0:
                self.start_turn_cycle('RIGHT')
                self.step_turn_cycle(dist)
                return

            # 处于中间带但还未进入 aligned_state 的情况
            # 保守处理：停住等待下一次稳定观测
            cmd = self.make_cmd(
                speed_kmh=0.0,
                angle_deg=0.0,
                dist_m=dist,
                pick_action=False,
                valid=True,
            )
            self.pub_bale_cmd.publish(cmd)
            return

        # --------------------------------------
        # 6) 已对准：低速靠近
        #    当前先保持简单，不做边走边小角修正
        # --------------------------------------
        if dist > self.approach_dist_m:
            self.exec_mode = 'APPROACH'
            cmd = self.make_cmd(
                speed_kmh=self.approach_speed_kmh,
                angle_deg=0.0,
                dist_m=dist,
                pick_action=False,
                valid=True,
            )
            self.pub_bale_cmd.publish(cmd)
            return

        # --------------------------------------
        # 7) 已对准且已到拾取距离：触发 pick_action
        # --------------------------------------
        self.exec_mode = 'IDLE'
        cmd_pick = self.make_cmd(
            speed_kmh=0.0,
            angle_deg=0.0,
            dist_m=dist,
            pick_action=True,
            valid=True,
        )
        self.pub_bale_cmd.publish(cmd_pick)
        self.pick_action_sent_time = now

        raw_angle = float(tgt.angle_deg)
        self.get_logger().info(
            f'Bale aligned and ready to pick: '
            f'dist={dist:.2f}m, raw_angle={raw_angle:.2f}deg, filt_angle={angle:.2f}deg'
        )


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
