# CAN反馈与去IMU说明

本文说明当前工程中“使用CAN报文获取车辆状态”的实现方式。重点覆盖两轮电机转速、VCU油门反馈、车速估计、轮速yaw rate，以及这些数据如何接入现有控制链路。

## 目标

当前方案车辆状态主要来自：

- RTK/GNSS：提供车辆位置和绝对航向。
- CAN反馈：提供左右轮电机转速、VCU油门AD值、VCU控制模式。

CAN反馈用于补充车辆控制需要的实际速度、左右轮速差和油门状态。RTK航向正常时，控制仍以RTK航向为主；RTK航向短时不可用时，`dr_odometry_node` 可以用左右轮速推导的 yaw rate 做短时积分。

## CAN报文

### 两轮电机转速反馈

| 方向 | CAN ID | 周期 | 数据 |
| --- | --- | --- | --- |
| 左电机 | `0x0CF11E05` | 50 ms | byte1=转速低字节，byte2=转速高字节 |
| 右电机 | `0x0CF11E06` | 50 ms | byte1=转速低字节，byte2=转速高字节 |

解码方式：

```text
motor_rpm = data[0] + data[1] * 256
```

单位为 `rpm`，分辨率为 `1 rpm/bit`，量程按协议为 `0-6000 rpm`。

### VCU油门反馈

| 方向 | CAN ID | 周期 | 数据 |
| --- | --- | --- | --- |
| 左电机 | `0x0CF1051E` | 100 ms | byte1=设定油门AD，byte2=CAN控制模式 |
| 右电机 | `0x0CF1061E` | 100 ms | byte1=设定油门AD，byte2=CAN控制模式 |

解码方式：

```text
throttle_v = data[0] * 5.0 / 255.0
can_control = data[1] == 1
```

其中 `data[0]` 的 `0-255` 对应 `0-5V`；`data[1] == 1` 表示CAN控制模式，其他值表示物理模式。

## 新增节点

新增节点：

```text
main/can_feedback_node.py
```

ROS可执行名：

```text
can_feedback_node
```

该节点负责从 SocketCAN 接收电机反馈报文，解码后发布为ROS话题。

## 发布话题

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| `/stack_can/feedback` | `stack_msgs/msg/CanFeedback` | CAN反馈汇总状态 |
| `/ground_speed_mps` | `std_msgs/msg/Float32` | 由左右轮转速估算的车辆速度，单位m/s |
| `/wheel_yaw_rate_rad_s` | `std_msgs/msg/Float32` | 由左右轮速差估算的yaw rate，单位rad/s |
| `/can/left_motor_rpm` | `std_msgs/msg/Float32` | 左电机转速 |
| `/can/right_motor_rpm` | `std_msgs/msg/Float32` | 右电机转速 |
| `/can/left_throttle_v` | `std_msgs/msg/Float32` | 左侧油门电压 |
| `/can/right_throttle_v` | `std_msgs/msg/Float32` | 右侧油门电压 |
| `/can/left_can_control` | `std_msgs/msg/Bool` | 左侧是否处于CAN控制模式 |
| `/can/right_can_control` | `std_msgs/msg/Bool` | 右侧是否处于CAN控制模式 |

## 速度和yaw rate计算

CAN报文给出的是电机转速。节点会先将电机转速换算成轮端线速度：

```text
wheel_rpm = motor_rpm * rpm_sign / motor_to_wheel_ratio
wheel_speed_mps = wheel_rpm * 2*pi*wheel_radius_m / 60
```

左右轮都有效时：

```text
ground_speed_mps = (left_wheel_speed_mps + right_wheel_speed_mps) / 2
yaw_rate_rad_s = (right_wheel_speed_mps - left_wheel_speed_mps) / wheel_track_m
```

只有单侧转速有效时，`ground_speed_mps` 使用该侧轮速，`yaw_rate_rad_s` 输出 `0`。

## 关键参数

参数位于：

```text
src/main/config/params.yaml
```

对应节点：

```yaml
can_feedback_node:
  ros__parameters:
    enable_can: true
    can_interface: "can0"
    can_extended: true

    left_rpm_can_id: 0x0CF11E05
    right_rpm_can_id: 0x0CF11E06
    left_throttle_can_id: 0x0CF1051E
    right_throttle_can_id: 0x0CF1061E

    wheel_radius_m: 0.315
    motor_to_wheel_ratio: 1.0
    wheel_track_m: 1.0
    left_rpm_sign: 1.0
    right_rpm_sign: 1.0
```

需要实车标定的参数：

- `wheel_radius_m`：车轮半径，单位m。
- `motor_to_wheel_ratio`：电机轴到轮端的减速比。如果报文已经是轮端转速，保持 `1.0`；如果是电机轴转速，应填写真实减速比。
- `wheel_track_m`：左右轮中心距，单位m，用于计算 yaw rate。
- `left_rpm_sign` / `right_rpm_sign`：转速方向符号。如果前进时某一侧速度为负，需要将对应符号改为 `-1.0`。

## 与现有控制链路的关系

### `traj_waypoint_follower`

循迹节点新增参数：

```yaml
actual_speed_topic: "/ground_speed_mps"
use_imu_fallback: false
```

含义：

- 实际速度反馈默认来自CAN计算出的 `/ground_speed_mps`。
- 当前仍建议使用RTK航向：

```yaml
use_rtk_heading: true
rtk_heading_topic: "/vehicle_heading_deg"
```

### `dr_odometry_node`

`dr_odometry_node` 现在可以同时使用：

- `/ground_speed_mps`：车辆线速度。
- `/vehicle_heading_deg`：RTK转换后的车辆航向。
- `/wheel_yaw_rate_rad_s`：由左右轮速差计算的 yaw rate。

正常情况下，RTK heading 是主航向来源。如果 heading 超时，节点会使用 `/wheel_yaw_rate_rad_s` 短时积分 yaw。

注意：轮速积分会漂移，只适合短时补偿，不应长期替代RTK航向。

## 启动方式

正式链路中已经加入：

```python
Node(
    package='main',
    executable='can_feedback_node',
    name='can_feedback_node',
    parameters=[params_file],
)
```

涉及的 launch 文件包括：

- `launch/demo.launch.py`
- `launch/ultrasonic.launch.py`
- `launch/docker_startup.launch.py`

## 编译

由于新增了消息类型 `stack_msgs/msg/CanFeedback.msg`，需要重新编译 `stack_msgs` 和 `main`：

```bash
cd /root/stack_can_ws
colcon build --packages-select stack_msgs main --symlink-install
source install/setup.bash
```

## 调试检查

启动后可检查话题是否存在：

```bash
ros2 topic list | grep -E "stack_can/feedback|ground_speed|wheel_yaw_rate|motor_rpm|throttle"
```

查看完整反馈：

```bash
ros2 topic echo /stack_can/feedback
```

查看车速：

```bash
ros2 topic echo /ground_speed_mps
```

查看左右轮转速：

```bash
ros2 topic echo /can/left_motor_rpm
ros2 topic echo /can/right_motor_rpm
```

查看油门反馈：

```bash
ros2 topic echo /can/left_throttle_v
ros2 topic echo /can/right_throttle_v
```

## 标定建议

1. 架空或低速直行时，确认左右电机转速是否都有数据。
2. 车辆直行前进时，确认 `/ground_speed_mps` 为正。
3. 如果前进时某一侧轮速为负，调整 `left_rpm_sign` 或 `right_rpm_sign`。
4. 用实测车速对比 `/ground_speed_mps`，修正 `wheel_radius_m` 或 `motor_to_wheel_ratio`。
5. 原地或小半径转向时，确认 `/wheel_yaw_rate_rad_s` 的方向是否与ROS yaw方向一致。
6. 确认 `/can/left_can_control` 和 `/can/right_can_control` 能正确反映VCU控制模式。

## 注意事项

- 当前 CAN 反馈节点只读取反馈报文，不参与控制命令仲裁。
- 控制命令发送仍由 `stack_can_executor` 负责。
- 如果同一个进程同时打开同一个 `can0` 接口收发 SocketCAN 报文通常是允许的，但车端实际运行时仍应观察是否有驱动或权限问题。
- 如果没有收到某侧转速，超过 `feedback_timeout_s` 后该侧会被标记为无效。
- 没有RTK heading时，仅靠轮速 yaw rate 推算航向会累积误差。
