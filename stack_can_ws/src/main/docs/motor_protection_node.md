# Motor Protection Node

`motor_protection_node` 用于在驱动电机出现疑似堵转、卡死或持续低速异常时触发整车急停保护。

当前车辆在实际测试中存在大量原地旋转动作。原地旋转时，地面阻力较大，左右电机容易出现“控制端持续给油，但电机转速很低甚至为 0”的情况。如果这种状态持续存在，电机和驱动器可能过热，甚至烧毁。

该节点通过监听 `can_feedback_node` 发布的电机反馈数据，判断是否存在堵转风险。一旦触发保护，节点发布 `/abort=True`，由 `stack_can_executor` 进入 ESTOP 状态。

## 文件位置

```text
stack_can_ws/src/main/main/motor_protection_node.py
```

## 启动方式

该节点已经注册到 `main` 包的 console script：

```bash
ros2 run main motor_protection_node --ros-args --params-file src/main/config/params.yaml
```

在整车流程中，节点由 `demo.launch.py` 启动：

```bash
ros2 launch main demo.launch.py
```

## 输入话题

### `/stack_can/feedback`

类型：

```text
stack_msgs/msg/CanFeedback
```

来源：

```text
can_feedback_node
```

主要使用字段：

```text
left_motor_rpm
right_motor_rpm
left_throttle_v
right_throttle_v
left_can_control
right_can_control
rpm_left_valid
rpm_right_valid
throttle_left_valid
throttle_right_valid
```

## 输出话题

### `/abort`

类型：

```text
std_msgs/msg/Bool
```

当检测到电机异常时发布：

```text
data: true
```

`stack_can_executor` 订阅 `/abort` 后会锁存 ESTOP，并向 VCU 输出急停状态。

## 保护逻辑

单侧电机满足以下条件时，认为该侧电机处于疑似堵转状态：

```text
该侧 rpm 反馈有效
AND 该侧 throttle 反馈有效
AND 该侧处于 CAN 控制模式
AND abs(throttle_v) >= throttle_high_v
AND abs(motor_rpm) <= stall_rpm_threshold
```

如果上述状态持续超过：

```text
stall_timeout_s
```

节点发布：

```text
/abort = true
```

左右任意一侧电机触发条件，都会触发整车急停。

## 参数说明

配置位置：

```text
stack_can_ws/src/main/config/params.yaml
```

示例：

```yaml
motor_protection_node:
  ros__parameters:
    feedback_topic: "/stack_can/feedback"
    abort_topic: "/abort"

    throttle_high_v: 0.8
    stall_rpm_threshold: 30.0
    stall_timeout_s: 1.0

    abort_on_feedback_invalid: false

    protect_left_motor: true
    protect_right_motor: true
    latch_abort: true
```

### `throttle_high_v`

油门电压阈值。

当油门电压绝对值大于等于该值时，认为系统正在持续给电机输出动力。

`can_feedback_node` 中油门反馈由原始值换算：

```text
throttle_v = throttle_raw * 5.0 / 255.0
```

推荐初始值：

```text
0.8
```

### `stall_rpm_threshold`

堵转转速阈值。

当电机转速绝对值小于等于该值时，认为电机处于低速或接近停止状态。

推荐初始值：

```text
30.0
```

### `stall_timeout_s`

堵转持续时间阈值。

只有当“高油门 + 低转速”持续超过该时间，才触发 `/abort`。这样可以避免车辆刚起步、换向、原地旋转瞬间阻力变化导致误触发。

推荐初始值：

```text
1.0
```

### `abort_on_feedback_invalid`

反馈无效时是否直接触发急停。

初期建议保持：

```text
false
```

原因是 CAN 反馈可能存在偶发丢帧。如果一丢帧就触发急停，车辆会频繁进入 ESTOP。等 CAN 反馈链路稳定后，可以再考虑改成 `true`。

### `protect_left_motor` / `protect_right_motor`

是否启用左、右电机保护。

一般保持：

```text
true
```

### `latch_abort`

触发一次 `/abort` 后，保护节点内部是否锁存，避免重复刷屏发布。

一般保持：

```text
true
```

## 推荐调参流程

先观察正常运行时反馈：

```bash
ros2 topic echo /stack_can/feedback
```

重点观察：

```text
left_motor_rpm
right_motor_rpm
left_throttle_v
right_throttle_v
left_can_control
right_can_control
```

然后进行原地旋转、低速转向、草地阻力较大场景测试。

如果误触发较多，可以调宽保护条件：

```yaml
throttle_high_v: 1.0
stall_rpm_threshold: 20.0
stall_timeout_s: 1.5
```

如果保护触发太慢，可以调严：

```yaml
throttle_high_v: 0.7
stall_rpm_threshold: 40.0
stall_timeout_s: 0.6
```

## 急停后的恢复

`motor_protection_node` 只负责发布 `/abort=True`。

急停锁存和恢复由 `stack_can_executor` 负责。当前逻辑中，`stack_can_executor` 收到 `/abort=True` 后会进入 ESTOP。解除 ESTOP 需要发布 `/reset_estop=True`，并且需要满足 `stack_can_executor` 内部的解锁条件。

手动发布恢复请求：

```bash
ros2 topic pub --once /reset_estop std_msgs/msg/Bool "{data: true}"
```

恢复前应确认：

1. 车辆已经停止；
2. 电机不再持续堵转；
3. 控制命令已经清零；
4. 周围环境安全。

## 手动测试

手动触发 `/abort`：

```bash
ros2 topic pub --once /abort std_msgs/msg/Bool "{data: true}"
```

观察 `/stack_can/status`：

```bash
ros2 topic echo /stack_can/status
```

观察保护节点日志：

```bash
ros2 run main motor_protection_node --ros-args --params-file src/main/config/params.yaml
```

## 注意事项

该保护逻辑主要针对“给油但电机不转”的堵转风险。它不是完整的电机热保护，也不能替代驱动器自身的过流、过温、欠压保护。

后续如果能从驱动器 CAN 报文中读取电流、温度、故障码，建议继续扩展保护条件：

```text
电机电流过大
驱动器温度过高
驱动器故障码非 0
CAN 反馈长时间丢失
左右电机转速严重不一致
```

当前版本优先解决最直接的风险：

```text
油门持续输出，但电机转速持续很低或为 0。
```
