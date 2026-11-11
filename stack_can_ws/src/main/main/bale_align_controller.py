#!/usr/bin/env python3
import math
import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import Bool, UInt8, String
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
        self.declare_parameter('retreat_max_duration_s', 2.0)

        # 目标有效范围
        self.declare_parameter('min_dist_m', 0.0)
        self.declare_parameter('max_dist_m', 15.0)

        # 目标超时
        self.declare_parameter('target_timeout_s', 1.0)

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
        self.retreat_max_duration_s = float(self.get_parameter('retreat_max_duration_s').value)
        if not math.isfinite(self.retreat_max_duration_s) or not 0.0 < self.retreat_max_duration_s <= 2.0:
            raise ValueError('retreat_max_duration_s must be between 0 and 2 seconds')

        self.min_dist_m = float(self.get_parameter('min_dist_m').value)
        self.max_dist_m = float(self.get_parameter('max_dist_m').value)

        self.target_timeout_s = float(self.get_parameter('target_timeout_s').value)
        self.pick_action_hold_s = float(self.get_parameter('pick_action_hold_s').value)

        # --------------------------------------------------
        # 订阅
        # --------------------------------------------------
        self.at_task_waiting = False
        self.drive_state = self.DS_PAUSED
        self.abort_req = False
        self.current_target: Optional[BaleTarget] = None
        self.last_target_recv_time = None
        self.no_target_since = None
        self.waiting_for_retry = False

        self.create_subscription(BaleTarget, '/bale_target', self.on_bale_target, 10)
        task_qos = QoSProfile(depth=1)
        task_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, '/at_task_waiting', self.on_task_waiting, task_qos)
        self.create_subscription(UInt8, '/drive_cmd', self.on_drive_cmd, 1)
        self.create_subscription(Bool, '/abort', self.on_abort, 1)
        self.create_subscription(Bool, '/reset_estop', self.on_reset_estop, 1)
        self.create_subscription(Bool, '/bale/retry', self.on_retry, 10)

        # --------------------------------------------------
        # 发布
        # --------------------------------------------------
        self.pub_bale_cmd = self.create_publisher(StackCommand, '/stack_cmd/bale', 1)

        qos_latched = QoSProfile(depth=1)
        qos_latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub_bale_active = self.create_publisher(Bool, '/bale_active', qos_latched)

        self.pub_task_done = self.create_publisher(Bool, '/task_done', 1)
        self.pub_diagnostics = self.create_publisher(String, '/bale_controller/status', 10)
        self.create_timer(0.5, self.publish_diagnostics)

        # --------------------------------------------------
        # 内部状态
        # --------------------------------------------------
        self.active = False
        self.pick_action_sent_time = None
        self.retreating = False
        self.retreat_started_monotonic = None
        self.retreat_limit_reached = False

        # 角度滤波
        self.filtered_angle_deg: Optional[float] = None

        # 滞回对准状态
        self.aligned_state = False
        self.align_confirm_count = 0
        self.align_confirm_last = None
        self.decision_samples = []
        self.decision_last = None
        self.decision_last_target_time = None
        self.last_turn_distance = 0.0
        self.align_exit_count = 0

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
            self.align_exit_count = 0
            self.current_target = None
            self.last_target_recv_time = now
            if not self.aligned_state:
                self.align_confirm_count = 0
                self.align_confirm_last = None
                self.decision_samples = []
                self.decision_last = None
                self.decision_last_target_time = None
            return

        self.current_target = msg
        self.last_target_recv_time = now

        raw_angle = float(msg.angle_deg)

        if self.filtered_angle_deg is None:
            self.filtered_angle_deg = raw_angle
        else:
            a = self.angle_lpf_alpha
            self.filtered_angle_deg = a * raw_angle + (1.0 - a) * self.filtered_angle_deg
        if self.aligned_state:
            if not math.isfinite(self.filtered_angle_deg):
                self.aligned_state = False
                self.align_exit_count = 0
                self.align_confirm_count = 0
                self.align_confirm_last = None
            elif abs(self.filtered_angle_deg) > self.align_exit_deg:
                self.align_exit_count += 1
                if self.align_exit_count >= 3:
                    self.aligned_state = False
                    self.align_exit_count = 0
                    self.align_confirm_count = 0
                    self.align_confirm_last = None
            else:
                self.align_exit_count = 0

    def on_task_waiting(self, msg: Bool):
        self.at_task_waiting = bool(msg.data)
        if not self.at_task_waiting:
            self.no_target_since = None
            self.waiting_for_retry = False
            self.retreat_started_monotonic = None
            self.retreat_limit_reached = False

    def on_retry(self, msg: Bool):
        if msg.data and self.at_task_waiting and (self.waiting_for_retry or self.retreat_limit_reached) and not self.abort_req and self.drive_state == self.DS_RUNNING:
            self.waiting_for_retry = False
            self.no_target_since = time.monotonic()
            self.get_logger().info('Manual bale retry requested')

    def on_drive_cmd(self, msg: UInt8):
        self.drive_state = int(msg.data)
        if self.drive_state == self.DS_ESTOP:
            self.abort_req = True

    def on_abort(self, msg: Bool):
        if bool(msg.data):
            self.abort_req = True
            self.drive_state = self.DS_ESTOP

    def on_reset_estop(self, msg: Bool):
        if bool(msg.data):
            self.abort_req = False

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

    def publish_diagnostics(self):
        now = self.now_s()
        target = self.current_target
        age = None if self.last_target_recv_time is None else max(0.0, now - self.last_target_recv_time)
        reasons = []
        if self.abort_req: reasons.append('abort_latched')
        if self.drive_state == self.DS_ESTOP: reasons.append('drive_estop')
        elif self.drive_state != self.DS_RUNNING: reasons.append('drive_not_running')
        if not self.at_task_waiting: reasons.append('not_at_task_waiting')
        if target is None: reasons.append('no_valid_target')
        elif not self.target_is_fresh(now): reasons.append('target_stale')
        if self.filtered_angle_deg is None: reasons.append('no_filtered_angle')
        if target is not None and not self.min_dist_m <= float(target.distance_m) <= self.max_dist_m:
            reasons.append('target_out_of_range')
        def finite(value):
            return value if value is None or math.isfinite(value) else None
        status = dict(active=self.active, abort_req=self.abort_req, drive_state=self.drive_state,
                      at_task_waiting=self.at_task_waiting, target_valid=target is not None,
                      target_age_s=finite(age), target_timeout_s=self.target_timeout_s,
                      distance_m=finite(float(target.distance_m)) if target else None,
                      raw_angle_deg=finite(float(target.angle_deg)) if target else None,
                      filtered_angle_deg=finite(self.filtered_angle_deg), aligned=self.aligned_state,
                      align_confirm_count=self.align_confirm_count,
                      align_exit_count=self.align_exit_count,
                      exec_mode=self.exec_mode, pick_active=self.pick_action_sent_time is not None,
                      waiting_for_retry=self.waiting_for_retry,
                      retreat_limit_reached=self.retreat_limit_reached,
                      no_target_s=0.0 if self.no_target_since is None else max(0.0, time.monotonic()-self.no_target_since),
                      blocking_reasons=reasons)
        self.pub_diagnostics.publish(String(data=json.dumps(status, allow_nan=False)))

    def reset_control_state(self):
        self.retreating = False
        self.aligned_state = False
        self.align_confirm_count = 0
        self.align_confirm_last = None
        self.align_exit_count = 0
        self.decision_samples = []
        self.decision_last = None
        self.decision_last_target_time = None
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

    def publish_pick_reset_cmd(self, reason: str):
        cmd = self.make_cmd(
            speed_kmh=0.0,
            angle_deg=0.0,
            dist_m=0.0,
            pick_action=False,
            valid=True,
        )
        self.pub_bale_cmd.publish(cmd)
        self.set_active(False)
        self.get_logger().warn(
            f'Abort bale sequence: {reason}; pick_action reset, /task_done not published'
        )

    def update_aligned_state(self, angle_abs: float, now: float):
        """
        双阈值滞回
        - 未对准 -> 每隔2秒确认一次，连续3次进入已对准
        - 已对准 -> abs(angle) >= exit 时退出已对准
        """
        if not self.aligned_state:
            if not math.isfinite(angle_abs) or angle_abs > self.align_enter_deg:
                self.align_confirm_count = 0
                self.align_confirm_last = None
            elif self.align_confirm_last is None or now - self.align_confirm_last >= 2.0:
                self.align_confirm_count += 1
                self.align_confirm_last = now
                if self.align_confirm_count >= 3:
                    self.aligned_state = True
        elif not math.isfinite(angle_abs):
            self.aligned_state = False
            self.align_exit_count = 0
            self.align_confirm_count = 0
            self.align_confirm_last = None

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

        if (self.at_task_waiting and self.retreat_started_monotonic is not None and
                time.monotonic() - self.retreat_started_monotonic >= self.retreat_max_duration_s):
            if not self.retreat_limit_reached:
                self.get_logger().warn('Retreat time limit reached; stopped, continuing pickup task')
            self.retreat_limit_reached = True

        # Count missing targets even if safety state currently blocks movement.
        if self.at_task_waiting and self.pick_action_sent_time is None:
            valid_target = self.target_is_fresh(now)
            if not valid_target and self.no_target_since is None:
                self.no_target_since = time.monotonic()
            elif valid_target and not self.waiting_for_retry:
                self.no_target_since = None
            if self.no_target_since is not None and time.monotonic() - self.no_target_since >= 10.0:
                self.waiting_for_retry = True

        # --------------------------------------
        # 0) 安全打断：abort / ESTOP 立即复位 pick_action，不发布 /task_done
        # --------------------------------------
        if self.abort_req or self.drive_state == self.DS_ESTOP:
            if self.active or self.pick_action_sent_time is not None:
                self.publish_pick_reset_cmd('abort or ESTOP')
            else:
                self.set_active(False)
            return

        # --------------------------------------
        # 1) 已进入 pick_action 保持阶段
        # --------------------------------------
        if self.pick_action_sent_time is not None:
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

        if self.at_task_waiting and self.waiting_for_retry:
            self.set_active(False)
            return

        if self.retreat_limit_reached:
            if self.target_is_fresh(now) and float(self.current_target.distance_m) >= self.too_close_dist_m:
                self.retreat_limit_reached = False
                self.retreat_started_monotonic = None
                self.retreating = False
            else:
                self.pub_bale_cmd.publish(self.make_cmd(0.0, 0.0, 0.0))
                self.set_active(True)
                return

        # A missing target for 10 seconds pauses automatic attempts until an operator retries or skips.
        if not self.target_is_fresh(now):
            # Keep the task in BALE. A turn already started may finish its bounded cycle;
            # otherwise hold still until target returns or the 10 second retry gate fires.
            self.decision_samples = []
            self.decision_last = None
            self.decision_last_target_time = None
            self.align_confirm_count = 0
            self.align_confirm_last = None
            self.aligned_state = False
            self.align_exit_count = 0
            if self.exec_mode in ('TURN_LEFT', 'TURN_RIGHT'):
                self.step_turn_cycle(self.last_turn_distance)
            else:
                self.pub_bale_cmd.publish(self.make_cmd(0.0, 0.0, 0.0))
            self.set_active(True)
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
        self.last_turn_distance = dist

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
        if self.retreating or self.retreat_started_monotonic is not None or dist < self.too_close_dist_m:
            self.retreating = True
            if self.retreat_started_monotonic is None:
                self.retreat_started_monotonic = time.monotonic()
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
            if not math.isfinite(angle):
                self.decision_samples = []
                self.decision_last = None
                self.decision_last_target_time = None
                self.align_confirm_count = 0
                self.pub_bale_cmd.publish(self.make_cmd(0.0, 0.0, dist))
                return
            if (self.decision_last is None or now - self.decision_last >= 2.0) and self.last_target_recv_time != self.decision_last_target_time:
                self.decision_samples.append(angle)
                self.decision_last = now
                self.decision_last_target_time = self.last_target_recv_time
                if max(self.decision_samples) - min(self.decision_samples) > 2.0:
                    self.decision_samples = [angle]
                self.align_confirm_count = len(self.decision_samples)
            if len(self.decision_samples) < 3:
                self.exec_mode = 'IDLE'
                self.pub_bale_cmd.publish(self.make_cmd(0.0, 0.0, dist))
                return
            samples = self.decision_samples
            self.decision_samples = []
            self.decision_last = None
            self.decision_last_target_time = None
            if all(abs(value) <= self.align_enter_deg for value in samples):
                self.aligned_state = True
            else:
                self.align_confirm_count = 0
            if not self.aligned_state and sum(samples) < 0.0:
                self.start_turn_cycle('LEFT')
                self.step_turn_cycle(dist)
                return

            if not self.aligned_state and sum(samples) > 0.0:
                self.start_turn_cycle('RIGHT')
                self.step_turn_cycle(dist)
                return

            # 处于中间带但还未进入 aligned_state 的情况
            # 保守处理：停住等待下一次稳定观测
            if not self.aligned_state:
                self.pub_bale_cmd.publish(self.make_cmd(0.0, 0.0, dist))
                return

        # --------------------------------------
        # 6) 已对准：低速靠近
        #    当前先保持简单，不做边走边小角修正
        # --------------------------------------
        if self.align_exit_count > 0:
            self.exec_mode = 'IDLE'
            self.pub_bale_cmd.publish(self.make_cmd(0.0, 0.0, dist))
            return
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
