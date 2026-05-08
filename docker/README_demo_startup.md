# demo.sh 启动流程说明

本文档用于帮助开发人员快速理解 `docker/demo.sh` 对整车 demo 的启动流程。

## 1. 启动入口

整车 demo 的推荐启动入口是：

```bash
./docker/demo.sh
```

该脚本会在宿主机上完成 Docker 容器启动、硬件设备映射、基础 ROS2 节点启动、启动前健康检查，并在检查通过后启动正式 demo。

整体流程如下：

```text
宿主机执行 docker/demo.sh
        |
        v
准备 Docker 运行环境
        |
        v
启动 nav_stack_ros2 容器
        |
        v
容器内启动基础节点 docker_startup.launch.py
        |
        v
执行 start_demo_after_health_check.sh
        |
        v
startup_health_check.py 检查关键传感器和 CAN 状态
        |
        +-- 检查通过 --> 启动 ros2 launch main demo.launch.py
        |
        +-- 检查失败 --> 不启动 demo，保留容器用于排查
```

## 2. `docker/demo.sh` 的主要作用

`docker/demo.sh` 负责完成宿主机到 Docker 容器的初始化工作。

主要功能包括：

1. 自动定位仓库根目录和 ROS2 工作空间。
2. 自动检测 `/dev/video0` 到 `/dev/video9` 摄像头设备，并映射进容器。
3. 配置 X11 显示权限，支持容器内运行 RViz 等图形程序。
4. 删除旧的 `nav_stack_ros2` 容器，避免容器名冲突。
5. 使用 `xinye30/ucar-jetson:nav-stack-ros2` 镜像启动新容器。
6. 使用 `--network host`，保证 ROS2 DDS、雷达 UDP、WebSocket 等通信正常。
7. 使用 `--privileged` 并挂载 `/dev`，允许容器访问 CAN、串口、雷达、摄像头等硬件。
8. 挂载宿主机的 `stack_can_ws` 到容器内 `/root/stack_can_ws`。
9. 容器启动后自动 source ROS2 和工作空间环境。
10. 后台启动基础节点。
11. 执行启动前健康检查，检查通过后启动正式 demo。

## 3. 容器内基础节点启动

`docker/demo.sh` 进入容器后，会启动：

```bash
ros2 launch main docker_startup.launch.py
```

该 launch 文件用于启动 demo 运行前必须先存在的基础节点。

当前基础节点包括：

| 节点 | 作用 |
| --- | --- |
| `nmea_bridge_node` | 从 RTK 串口读取 NMEA 数据，发布 `/fix` 和 `/heading_deg` |
| `server_all_ws.py` | 启动 WebSocket 服务，用于远程通信和状态交互 |
| `can_feedback_node` | 读取 VCU/CAN 反馈，发布 `/stack_can/feedback`、电机转速、油门反馈等状态 |
| `ultrasonic` | 读取超声波传感器串口数据，发布 `/ultrasonic_distances` 和障碍物状态 |
| `rtk_center_from_nmea` | 根据 RTK 主天线位置和航向角计算车辆中心位置 |
| `rsview` | 启动 RoboSense 激光雷达驱动，发布 `/rslidar_points` |

基础节点日志默认写入：

```bash
/tmp/docker_startup.log
```

## 4. 启动前健康检查

基础节点启动后，脚本会执行：

```bash
/root/stack_can_ws/script/start_demo_after_health_check.sh
```

该脚本会调用：

```bash
/root/stack_can_ws/script/startup_health_check.py
```

当前健康检查项目包括：

| 检查项 | 话题 | 说明 |
| --- | --- | --- |
| RTK 定位 | `/fix` | 检查 RTK/GPS 是否有有效定位 |
| RTK 航向角 | `/heading_deg` | 检查双天线航向角是否正常发布 |
| 激光雷达点云 | `/rslidar_points` | 检查雷达是否正常发布点云 |
| 超声波 | `/ultrasonic_distances` | 检查超声波节点是否正常发布距离数据 |
| CAN 状态 | `/stack_can/feedback` | 检查 VCU/CAN 反馈是否正常 |

默认启动检查超时时间为：

```bash
STARTUP_HEALTH_TIMEOUT=5.0
```

默认运行期健康检查超时时间为：

```bash
RUNTIME_HEALTH_TIMEOUT=2.0
```

可以通过环境变量修改：

```bash
STARTUP_HEALTH_TIMEOUT=8.0 ./docker/demo.sh
```

## 5. CAN 状态检查逻辑

CAN 检查不仅判断 `/stack_can/feedback` 是否有发布，还会判断反馈内容是否有效。

只要以下任意字段为 `true`，就认为 CAN 反馈有效：

```text
rpm_left_valid
rpm_right_valid
throttle_left_valid
throttle_right_valid
```

如果 `/stack_can/feedback` 有发布，但这些字段全部为 `false`，说明 CAN 节点虽然运行了，但没有收到有效的 VCU 反馈帧。

常见原因包括：

1. `can0` 未启动。
2. VCU 未上电。
3. CAN-H / CAN-L 接线异常。
4. CAN 终端电阻异常。
5. CAN ID 与参数配置不匹配。
6. VCU 当前没有发送对应反馈帧。

## 6. 超声波检查逻辑

超声波节点发布：

```bash
/ultrasonic_distances
```

消息类型为 `Float32MultiArray`。

健康检查要求至少有一路距离值大于 `0.0`，才认为超声波链路有效。

如果检查失败，常见原因包括：

1. `ultrasonic` 节点未启动。
2. 串口设备不存在。
3. 超声波传感器未上电。
4. 串口权限不足。
5. 波特率配置错误。
6. 串口号配置错误。

默认串口配置在 `docker_startup.launch.py` 中：

```python
'port': '/dev/ttyTHS0',
'baudrate': 9600,
```

如果实际设备不是 `/dev/ttyTHS0`，需要修改该参数。

## 7. 检查通过后的正式 demo 启动

如果所有健康检查通过，脚本会启动：

```bash
ros2 launch main demo.launch.py
```

也可以通过环境变量覆盖默认 demo 启动命令：

```bash
DEMO_LAUNCH="ros2 launch main demo.launch.py some_arg:=value" ./docker/demo.sh
```

## 8. 检查失败后的行为

如果健康检查失败：

1. `demo.launch.py` 不会启动。
2. 基础节点仍然保留在后台运行。
3. 容器不会立即退出。
4. 开发人员可以进入当前容器继续排查问题。

查看健康检查日志：

```bash
cat /tmp/startup_health_check.log
```

查看基础节点启动日志：

```bash
cat /tmp/docker_startup.log
```

修复问题后，可以在当前容器内重新执行：

```bash
/root/stack_can_ws/script/start_demo_after_health_check.sh
```

## 9. 常用排查命令

查看关键话题是否存在：

```bash
ros2 topic list
```

检查 RTK：

```bash
ros2 topic echo /fix
ros2 topic echo /heading_deg
```

检查雷达：

```bash
ros2 topic echo /rslidar_points
```

检查超声波：

```bash
ros2 topic echo /ultrasonic_distances
ros2 topic echo /ultrasonic_obstacle_near
```

检查 CAN 反馈：

```bash
ros2 topic echo /stack_can/feedback
ros2 topic echo /can/left_motor_rpm
ros2 topic echo /can/right_motor_rpm
```

检查 CAN 接口：

```bash
ip link show can0
candump can0
```

如果 `candump can0` 没有数据，优先检查 VCU 上电、CAN 接线和波特率配置。

## 10. 开发注意事项

1. `docker_startup.launch.py` 只放基础硬件节点，不放完整业务逻辑。
2. `startup_health_check.py` 只负责判断 demo 是否具备启动条件。
3. `demo.launch.py` 是正式业务 demo 的入口。
4. 新增关键传感器时，应同步加入：
   - 基础节点启动；
   - 健康检查订阅；
   - 失败提示信息；
   - 本 README 文档。
5. 启动检查失败时，不应强行启动正式 demo，否则车辆可能在缺少定位、雷达、超声波或 CAN 状态的情况下运行。

