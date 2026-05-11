# Trajectory Follower 与 Bale Align Controller 控制逻辑说明

本文档用于说明当前分支中两个核心节点的协作关系：

- `traj_waypoint_follower.py`
- `bale_align_controller.py`

这两个节点共同完成：

1. CSV 路径跟随；
2. 到达草捆捡拾任务点；
3. 切换到草捆对准控制；
4. 完成捡拾动作；
5. 继续路径跟随；
6. 到达卸货点并执行卸货；
7. 在 `abort / ESTOP` 时安全打断动作。

---

# 1. 总体控制链路

`demo.launch.py` 启动后，主要控制链路如下：

```text
traj_waypoint_follower
        ↓
/stack_cmd/traj
        ↓
stack_can_executor
        ↓
CAN / VCU
```

草捆捡拾时：

```text
bale_detector
        ↓
/bale_target
        ↓
bale_align_controller
        ↓
/stack_cmd/bale
        ↓
stack_can_executor
        ↓
CAN / VCU
```

`stack_can_executor` 中的控制优先级为：

```text
TELEOP > BALE > TRAJ
```

因此，当 `/bale_active=True` 且 `/stack_cmd/bale` 有效时，车辆控制权会从普通循迹命令切换到草捆对准命令。

---

# 2. traj_waypoint_follower.py 功能说明

`traj_waypoint_follower.py` 是路径跟随与任务点管理节点。

它不直接发送 CAN，而是发布：

```text
/stack_cmd/traj
```

由 `stack_can_executor` 转换为 CAN 指令。

---

# 3. traj_waypoint_follower.py 输入

## 3.1 CSV 路径文件

通过参数：

```text
path_csv
```

读取路径点。

CSV 格式：

```text
idx, latitude, longitude, heading_deg, point_type
```

其中：

| point_type | 含义 |
|---|---|
| 0 | 普通路径点 |
| 1 | 任务点，通常为草捆捡拾点 |
| 2 | 卸货点 |

如果启用：

```text
legacy_final_task_unload = True
```

则最后一个 `type=1` 点仍会被兼容为卸货点。

---

## 3.2 订阅 Topic

| Topic | 类型 | 作用 |
|---|---|---|
| `/fix` 或配置的 `gps_topic` | `sensor_msgs/NavSatFix` | GPS 位置 |
| `/gps/heading_deg` | `std_msgs/Float32` | RTK 航向 |
| `/imu/data` | `sensor_msgs/Imu` | IMU fallback，仅在关闭 RTK heading 且启用 IMU fallback 时使用 |
| `/ground_speed_mps` | `std_msgs/Float32` | 实际速度，用于 dead reckoning |
| `/drive_cmd` | `std_msgs/UInt8` | 启停状态 |
| `/abort` | `std_msgs/Bool` | 急停 |
| `/task_done` | `std_msgs/Bool` | 草捆捡拾完成 |
| `/restart_path` | `std_msgs/Bool` | 重新读取 CSV 路径 |

---

# 4. traj_waypoint_follower.py 输出

| Topic | 类型 | 作用 |
|---|---|---|
| `/stack_cmd/traj` | `stack_msgs/StackCommand` | 普通路径跟随控制命令 |
| `/at_task_waiting` | `std_msgs/Bool` | 是否到达任务点并等待任务完成 |
| `global_path` | `nav_msgs/Path` | RViz 显示全局路径 |
| `traj_path` | `nav_msgs/Path` | RViz 显示实际轨迹 |

---

# 5. 路径跟随逻辑

节点启动后会：

1. 读取 CSV；
2. 以第一个路径点作为 ENU 原点；
3. 将 GPS 经纬度转换为局部 ENU 坐标；
4. 将 CSV heading 转换为 ENU yaw；
5. 根据当前位置与当前路径段计算：
   - 横向误差；
   - 航向误差；
   - 到目标点距离；
   - 是否接近拐角；
   - 是否到达任务点或卸货点；
6. 生成离散车辆控制命令。

输出控制量包括：

```text
pre_speed_kmh
angle_deg
dist_to_target_m
pick
unload
dump
pick_action
valid
```

---

# 6. 普通路径点处理

普通路径点 `type=0` 只参与路径跟随。

车辆接近当前路径段终点后，节点会自动切换到下一段。

---

# 7. 草捆捡拾点处理

草捆捡拾点对应：

```text
point_type = 1
```

到达判断主要包含：

```text
距离满足 wp_reached_dist
航向误差满足 wp_heading_tol_deg
```

默认参数：

```text
wp_reached_dist = 0.8 m
wp_heading_tol_deg = 10 deg
```

当车辆到达草捆任务点后，`traj_waypoint_follower.py` 会：

```text
waiting_for_task = True
drive_state = PAUSED
```

并发布：

```text
/at_task_waiting = True
/stack_cmd/traj = 停车
```

此时普通循迹暂停，等待 `bale_align_controller.py` 完成草捆对准和捡拾。

---

# 8. 卸货点处理

卸货点对应：

```text
point_type = 2
```

到达卸货点后，节点进入：

```text
unload_mode = True
```

卸货流程：

```text
阶段 1：发布 unload=True，持续 unload_hold_time_s
阶段 2：发布 unload=False，持续 unload_reset_time_s
阶段 3：调用 finish_unload()
```

默认时间：

```text
unload_hold_time_s = 30.0
unload_reset_time_s = 2.0
```

如果是中间卸货点，卸货完成后继续循迹。

如果是最后一个卸货点，卸货完成后停车。

---

# 9. traj_waypoint_follower.py 的 abort / ESTOP 处理

当前版本新增了：

```python
self.abort_req = False
```

当收到：

```text
/abort = True
```

或：

```text
/drive_cmd = 2
```

时：

```python
self.abort_req = True
self.drive_state = DS_ESTOP
```

---

# 10. 卸货期间的 abort 打断逻辑

当前版本重点修复了卸货期间 abort 不能打断的问题。

在 `unload_mode` 中，如果检测到：

```text
abort_req == True
```

或：

```text
drive_state == DS_ESTOP
```

会立即执行：

```text
unload=False
unload_mode=False
unload_end_time=0
unload_reset_end_time=0
unload_target_idx=-1
```

同时不会调用：

```python
finish_unload()
```

也不会将当前卸货点加入：

```python
completed_unload_indices
```

因此该卸货点不会被误判为完成。

恢复后，如果车辆仍处于该卸货点附近，会重新执行完整卸货流程。

---

# 11. bale_align_controller.py 功能说明

`bale_align_controller.py` 是草捆末端对准与捡拾动作控制节点。

它只有在车辆到达任务点后才会激活。

激活条件：

```text
/drive_cmd == RUNNING
/at_task_waiting == True
/bale_target 有效且未超时
```

---

# 12. bale_align_controller.py 输入

| Topic | 类型 | 作用 |
|---|---|---|
| `/bale_target` | `stack_msgs/BaleTarget` | 草捆检测结果 |
| `/at_task_waiting` | `std_msgs/Bool` | traj 节点是否到达任务点 |
| `/drive_cmd` | `std_msgs/UInt8` | 启停状态 |
| `/abort` | `std_msgs/Bool` | 急停 |
| `/reset_estop` | `std_msgs/Bool` | 清除本节点 abort 标志 |

---

# 13. bale_align_controller.py 输出

| Topic | 类型 | 作用 |
|---|---|---|
| `/stack_cmd/bale` | `stack_msgs/StackCommand` | 草捆对准控制命令 |
| `/bale_active` | `std_msgs/Bool` | 是否由草捆对准节点接管 |
| `/task_done` | `std_msgs/Bool` | 捡拾动作完成通知 |

---

# 14. 草捆对准流程

`bale_align_controller.py` 收到 `/bale_target` 后，会读取：

```text
valid
distance_m
angle_deg
```

控制逻辑：

```text
目标无效或超时：
    bale_active=False

目标有效：
    根据 angle_deg 判断草捆在左侧还是右侧

未对准：
    固定周期原地转向修正

已对准：
    低速靠近草捆

距离满足 approach_dist_m：
    触发 pick_action
```

---

# 15. 固定周期转向控制

对准阶段不是连续高频微调，而是固定周期控制：

```text
PULSE 阶段：连续发送转向命令
PAUSE 阶段：发送中性命令
一个周期完成后重新观察目标角度
```

默认参数：

```text
turn_frame_period_s = 0.10
turn_pulse_frames = 5
turn_pause_frames = 1
turn_cmd_angle_deg = 10.0
```

方向约定：

```text
angle_deg < 0：目标在左，左转
angle_deg > 0：目标在右，右转
```

如果现场发现方向相反，只需要交换 LEFT / RIGHT。

---

# 16. 捡拾动作流程

当草捆已经对准，并且距离小于等于：

```text
approach_dist_m
```

节点会发布：

```text
pick_action=True
```

并保持：

```text
pick_action_hold_s
```

默认：

```text
pick_action_hold_s = 30.0
```

保持结束后：

```text
pick_action=False
/task_done=True
/bale_active=False
```

`traj_waypoint_follower.py` 收到 `/task_done=True` 后退出任务等待，继续执行后续路径。

---

# 17. bale_align_controller.py 的 abort / ESTOP 处理

当前版本新增了：

```python
self.abort_req = False
```

当收到：

```text
/abort=True
```

或：

```text
/drive_cmd=2
```

时：

```python
abort_req = True
drive_state = DS_ESTOP
```

主循环最前面会优先检查：

```python
if self.abort_req or self.drive_state == self.DS_ESTOP:
```

如果当前正在对准、靠近或执行 `pick_action`，会立即发布：

```text
pick_action=False
/stack_cmd/bale = 停车命令
/bale_active=False
```

并且不会发布：

```text
/task_done=True
```

这保证了 abort 不会误判捡拾完成。

---

# 18. abort 后恢复策略

当前策略是：

```text
abort 后不判定动作完成
abort 后不从剩余时间继续执行
abort 后恢复时重新执行当前动作
```

原因：

1. 捡拾和卸货是机械动作；
2. abort 发生时无法可靠知道机械动作执行到哪一步；
3. 直接判定完成可能导致漏捡或漏卸；
4. 从中断时间继续执行也不可靠；
5. 重新执行完整动作最安全、最容易现场验证。

---

# 19. 完整任务流程

```text
CSV 路径开始
    ↓
traj_waypoint_follower 跟随普通路径点
    ↓
到达 type=1 草捆任务点
    ↓
/at_task_waiting=True
    ↓
bale_align_controller 激活
    ↓
/bale_active=True
    ↓
stack_can_executor 选择 BALE 模式
    ↓
草捆对准
    ↓
低速靠近
    ↓
pick_action=True 保持 30s
    ↓
pick_action=False
    ↓
/task_done=True
    ↓
traj_waypoint_follower 继续路径
    ↓
到达 type=2 卸货点
    ↓
unload=True 保持 30s
    ↓
unload=False 复位 2s
    ↓
继续路径或最终停车
```

---

# 20. abort 场景总结

| 场景 | 当前处理 |
|---|---|
| 普通循迹时 abort | 停车，进入 ESTOP |
| 草捆对准时 abort | 停止对准，`bale_active=False` |
| pick_action 期间 abort | `pick_action=False`，不发 `/task_done` |
| 卸货期间 abort | `unload=False`，不调用 `finish_unload()` |
| abort 恢复后 | 当前任务重新执行 |
| 是否自动判定任务完成 | 否 |
| 是否从中断时间继续 | 否 |

---

# 21. 开发注意事项

1. `traj_waypoint_follower.py` 不直接控制 CAN。
2. `bale_align_controller.py` 不负责路径跟随。
3. 控制权切换由 `/bale_active` 和 `stack_can_executor` 的优先级完成。
4. `/task_done` 只能由 `bale_align_controller.py` 在完整捡拾完成后发布。
5. abort 后不要发布 `/task_done`。
6. 卸货 abort 后不要调用 `finish_unload()`。
7. 中断恢复后，默认重新执行当前捡拾或卸货动作。
8. 如果后续需要“人工确认后继续”，建议新增独立 topic，例如 `/resume_task_action`，不要复用 `/drive_cmd`。
