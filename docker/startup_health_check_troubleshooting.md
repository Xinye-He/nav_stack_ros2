# 启动前健康检查失败排障说明

本文档用于说明农业无人车软件系统在启动前健康检查失败后，针对不同报错信息应执行的外设检查和恢复措施。

启动流程中，系统会先启动基础节点，然后运行启动前健康检查脚本：

```bash
/root/stack_can_ws/script/start_demo_after_health_check.sh
```

如果健康检查失败，`demo.launch.py` 不会启动。此时基础节点通常仍在后台运行，可以先查看日志，再根据具体报错检查对应外设。

## 1. 日志查看

启动失败后，优先查看以下日志：

```bash
cat /tmp/health_monitor_startup.log
cat /tmp/docker_startup.log
```

也可以直接重新执行健康检查：

```bash
/root/stack_can_ws/script/start_demo_after_health_check.sh
```

健康检查主要检查以下话题：

| 检查对象 | ROS2 话题 | 对应外设 |
|---|---|---|
| RTK/GPS 定位 | `/fix` | RTK 主机、GNSS 天线、串口/NMEA |
| RTK 航向角 | `/heading_deg` | 双天线 RTK、THS/HDT 航向输出 |
| 激光雷达点云 | `/rslidar_points` | RoboSense E1R、网线、IP、端口 |
| 超声波距离 | `/ultrasonic_distances` | 一拖四超声波模块、串口、电源 |
| CAN 反馈 | `/stack_can/feedback` | VCU、CAN0、CAN 接线、终端电阻 |

---

## 2. RTK/GPS 无有效 `/fix` 数据

### 典型报错

```text
RTK/GPS 无有效 /fix 数据。
请检查 RTK 是否上电、天线、串口、NMEA GGA/RMC 输出。
```

### 含义

系统没有收到有效的 GNSS 定位数据，或者收到的数据状态为无效定位。健康检查中，`/fix` 需要有有效的 `NavSatFix` 数据。

### 外设检查步骤

#### 2.1 检查 NMEA 是否有原始数据

可以用 `minicom` 或 `cat` 检查串口是否有 NMEA 数据输出。

例如：

```bash
minicom -D /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 -b 115200
```

正常情况下应能看到类似：

```text
$GNGGA,...
$GNRMC,...
```

如果完全没有输出，重点检查：

- RTK 是否上电；
- 串口号是否正确；
- 波特率是否正确；
- TX/RX 是否接反；
- RTK 是否开启 NMEA 输出。


## 3. RTK 航向角无有效 `/heading_deg` 数据

### 典型报错

```text
RTK 航向角无有效 /heading_deg 数据。
请检查 THS/HDT 输出、双天线定向、RTK 配置。
```

### 含义

系统没有收到车辆航向角。路径跟踪需要车辆当前航向，因此 `/heading_deg` 是正式自动作业前的关键输入。

### 外设检查步骤

#### 3.1 检查 RTK 是否输出航向语句

航向角常见 NMEA 语句包括：

```text
$GNHDT
$GNTHS
```

使用串口查看：

```bash
minicom -D /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 -b 115200
```

检查是否有类似：

```text
$GNTHS,...
```

如果只有 `$GNGGA`、`$GNRMC`，没有航向语句，需要进入 RTK 配置软件开启 THS/HDT 输出。


#### 3.4 检查天线方向是否反向

如果 `/heading_deg` 有数据，但车辆实际方向与显示方向相反或偏差很大，需要检查：

- 主副天线前后是否装反；
- `heading_from_north_cw` 参数是否符合当前约定；
- `main_to_center_x`、`main_to_center_y` 是否符合实车安装；
- RTK 模块输出的是航向角还是方位角；
- 车辆坐标系是否仍为 x 前向、y 左向。

---

## 4. 激光雷达无有效 `/rslidar_points` 点云

### 典型报错

```text
激光雷达无有效 /rslidar_points 点云。
请检查雷达是否上电、网线、主机 IP、msop/difop 端口、雷达 IP。
```

### 含义

系统没有收到 RoboSense E1R 的点云数据。草捆检测依赖 `/rslidar_points`，因此雷达点云缺失时正式 demo 不会启动。

### 外设检查步骤

#### 4.1 检查雷达是否上电

检查内容：

- 雷达电源是否接通；
- 雷达指示灯是否正常；
- 电源电压是否满足要求；
- 雷达是否有启动时间，刚上电可能需要等待数秒；
- 航空插头或电源接头是否松动。

#### 4.2 检查网线连接

检查 Jetson 网口状态：

```bash
ip addr show eth0
ip link show eth0
```

如果网口没有 `LOWER_UP`，说明物理链路可能未连接。

重点检查：

- 网线是否插紧；
- 雷达和 Jetson 是否通过正确网口连接；
- 交换机是否上电；
- 网口灯是否闪烁；
- 网线是否损坏。

#### 4.3 检查主机 IP

雷达通常要求主机 IP 与雷达 IP 在同一网段。

查看当前 IP：

```bash
ip -br addr
```

如果雷达 IP 例如为：

```text
192.168.1.200
```

则 Jetson 网口可以配置为：

```bash
sudo ip addr add 192.168.1.102/24 dev eth0
sudo ip link set eth0 up
```

注意不要重复添加多个冲突 IP。

#### 4.4 检查是否收到 UDP 数据

RoboSense 雷达一般通过 UDP 发送 MSOP/DIFOP 数据。可用 `tcpdump` 检查：

```bash
sudo tcpdump -ni eth0 udp
```

也可以按常见端口过滤：

```bash
sudo tcpdump -ni eth0 udp port 6699 or udp port 7788
```

如果能看到 UDP 包，说明雷达数据到达主机，问题可能在驱动配置。

如果完全没有 UDP 包，优先检查：

- 雷达是否上电；
- 网线是否正常；
- IP 是否同网段；
- 雷达目标 IP 是否设置为 Jetson IP；
- 防火墙或网络配置是否阻止 UDP。

#### 4.5 检查 ROS2 点云话题

```bash
ros2 topic list | grep rslidar
ros2 topic hz /rslidar_points
```

正常情况下 `/rslidar_points` 应持续发布。

如果 `tcpdump` 能收到 UDP，但 `/rslidar_points` 没有数据，检查：

- `rsview` 节点是否启动；
- 雷达驱动参数中的雷达型号是否正确；
- MSOP/DIFOP 端口是否与雷达配置一致；
- 容器是否使用 host 网络；
- 驱动是否绑定到了正确网卡。

---

## 5. 超声波无有效 `/ultrasonic_distances` 数据

### 典型报错

```text
超声波无有效 /ultrasonic_distances 数据。
请检查 ultrasonic 节点是否启动、串口设备是否存在、传感器是否上电、波特率是否正确。
```

### 含义

系统没有收到四路超声波距离数组。健康检查只要求 `/ultrasonic_distances` 至少有 4 路数据，不判断距离是否为 0。

也就是说：

```text
[0.0, 0.0, 0.0, 0.0]
```

在启动前健康检查中也可以视为“有数据”。真正的近距离障碍判断由超声波节点或安全仲裁逻辑处理。

### 外设检查步骤

#### 5.1 检查超声波模块是否上电

检查内容：

- 一拖四超声波模块是否通电；
- 每一路探头是否连接；
- 电源电压是否正常；
- 传感器接头是否松动；
- 模块指示灯是否正常。

#### 5.2 检查串口设备是否存在

```bash
ls /dev/ttyUSB*
ls /dev/ttyTHS*
```

如果使用 USB 转串口，可以插拔后查看：

```bash
dmesg | tail -n 50
```

如果没有出现串口设备，重点检查：

- USB 转串口是否识别；
- USB 线是否正常；
- Docker 是否挂载 `/dev`；
- 当前用户或容器是否有串口访问权限。

#### 5.3 检查串口波特率

超声波模块需要使用与代码配置一致的波特率。

可以用：

```bash
minicom -D /dev/ttyUSB0 -b 9600
```

或根据实际配置改为：

```bash
minicom -D /dev/ttyUSB0 -b 115200
```

如果串口能打开但数据乱码，通常是波特率不匹配。

#### 5.4 检查 ROS2 话题

```bash
ros2 topic echo /ultrasonic_distances
```

正常情况下应输出：

```text
data:
- 1.23
- 1.18
- 0.0
- 0.0
```

如果话题不存在：

```bash
ros2 node list | grep ultrasonic
```

检查 `ultrasonic` 节点是否启动。

如果节点存在但话题无数据，重点检查：

- 串口路径是否配置正确；
- 波特率是否正确；
- 模块通信协议是否与代码匹配；
- 传感器是否返回完整四路数据。

---

## 6. CAN 状态无有效 `/stack_can/feedback` 数据

### 典型报错

```text
CAN 状态无有效 /stack_can/feedback 数据。
请检查 can_feedback_node 是否启动、can0 是否 up、VCU 是否上电、CAN 接线和终端电阻。
```

### 含义

系统没有收到 VCU/CAN 反馈。路径跟踪、底盘控制、电机堵转保护都依赖 CAN 反馈状态，因此 CAN 无反馈时不应启动自动作业。

### 外设检查步骤

#### 6.1 检查 CAN 设备是否存在

```bash
ip link show can0
```

如果没有 `can0`，说明系统未识别 CAN 设备。

可能原因：

- CAN 适配器未插入；
- SocketCAN 驱动未加载；
- 设备名不是 `can0`；
- Docker 未挂载网络设备；
- Jetson CAN 引脚或外置 CAN 模块配置异常。

#### 6.2 检查 can0 是否 up

```bash
ip -d link show can0
```

如果状态是 `DOWN`，需要启动：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up
```

具体波特率必须与 VCU 一致。

#### 6.3 检查 VCU 是否上电

检查内容：

- VCU 是否通电；
- 底盘总电源是否打开；
- 急停按钮是否释放；
- VCU 指示灯是否正常；
- VCU 是否处于可通信状态；
- 电机驱动器是否上电。

如果 VCU 未上电，`can0` 可以 up，但不会收到任何反馈帧。

#### 6.4 检查 CAN 接线

CAN 总线至少需要：

- CAN_H 对 CAN_H；
- CAN_L 对 CAN_L；
- GND 共地；
- 总线两端 120 Ω 终端电阻。

断电状态下，可以用万用表测 CAN_H 与 CAN_L 之间电阻。

正常双端终端电阻约为：

```text
60 Ω
```

如果测得约 `120 Ω`，可能只有一端终端。

如果接近无穷大，可能没有终端或线断。

如果接近 `0 Ω`，可能短路。

#### 6.5 使用 candump 检查原始 CAN 帧

```bash
candump can0
```

如果能看到 VCU 持续发送反馈帧，说明 CAN 物理层基本正常。

如果完全没有帧：

- 检查 VCU 是否上电；
- 检查波特率；
- 检查 CAN_H/CAN_L 是否接反；
- 检查终端电阻；
- 检查急停或 VCU 模式是否阻止反馈输出。

#### 6.6 检查 ROS2 CAN 反馈话题

```bash
ros2 topic echo /stack_can/feedback
```

如果 `candump can0` 有帧，但 `/stack_can/feedback` 没有数据，重点检查：

- `can_feedback_node` 是否启动；
- CAN ID 是否与 VCU 实际反馈帧一致；
- `params.yaml` 中反馈 CAN ID 是否正确；
- `stack_msgs` 是否编译正确；
- 节点是否报解析异常。

---

## 7. CAN 话题有发布，但左右电机 RPM/油门反馈均无效

### 典型报错

```text
CAN 状态话题 /stack_can/feedback 有发布，但左右电机 RPM/油门反馈均无效。
请检查 VCU 是否发送反馈帧，以及 CAN ID 是否与参数配置匹配。
```

### 含义

`can_feedback_node` 正在发布 `/stack_can/feedback`，但是消息中的有效标志均为无效。例如：

```text
rpm_left_valid: false
rpm_right_valid: false
throttle_left_valid: false
throttle_right_valid: false
```

这通常说明 ROS2 节点在运行，但没有解析到期望的 VCU 反馈帧。

### 外设检查步骤

#### 7.1 先确认原始 CAN 帧

```bash
candump can0
```

记录实际出现的 CAN ID，例如：

```text
can0  201   [8]  ...
can0  202   [8]  ...
```

#### 7.2 对照参数文件

打开参数配置：

```bash
vim /root/stack_can_ws/src/main/config/params.yaml
```

检查 `can_feedback_node` 相关配置：

- 左电机 RPM 反馈 ID；
- 右电机 RPM 反馈 ID；
- 左油门反馈 ID；
- 右油门反馈 ID；
- 数据字节序；
- 缩放系数；
- 是否启用了对应 valid 判断。

如果 VCU 实际 CAN ID 与参数不一致，节点会收到 CAN 帧，但无法解析为有效反馈。

#### 7.3 检查 VCU 协议版本

如果近期更换过 VCU、驱动器或供应商程序，需要确认：

- CAN ID 是否变化；
- 数据长度是否变化；
- RPM 字段位置是否变化；
- 油门反馈字段位置是否变化；
- 是否需要发送使能帧后 VCU 才反馈完整状态。

---

## 8. 多个检查项同时失败

### 典型现象

日志中同时出现多个错误，例如：

```text
RTK/GPS 无有效 /fix 数据。
RTK 航向角无有效 /heading_deg 数据。
激光雷达无有效 /rslidar_points 点云。
CAN 状态无有效 /stack_can/feedback 数据。
```

### 优先排查顺序

建议按以下顺序检查：

1. Docker 和基础节点是否正常启动；
2. `/dev` 是否正确挂载到容器；
3. 电源系统是否正常；
4. RTK、雷达、VCU、超声波是否全部上电；
5. 网络接口和串口设备是否存在；
6. `docker_startup.launch.py` 是否真正运行；
7. 各传感器话题是否存在；
8. 各话题是否有数据。

### 检查基础节点是否启动

```bash
ros2 node list
```

如果几乎没有节点，先查看：

```bash
cat /tmp/docker_startup.log
```

### 检查话题列表

```bash
ros2 topic list
```

如果健康检查所需话题都不存在，通常不是外设单独故障，而是基础 launch 没有正常启动。

---

## 9. 修复后重新启动

修复对应外设后，不一定需要重启整个 Docker 容器。可以在当前容器中重新执行：

```bash
/root/stack_can_ws/script/start_demo_after_health_check.sh
```

如果基础节点也异常，建议重新启动 Docker demo：

```bash
cd ~/nav_stack_ros2
./docker/demo.sh
```

---

## 10. 快速排障命令汇总

### 查看健康检查日志

```bash
cat /tmp/health_monitor_startup.log
cat /tmp/docker_startup.log
```

### 查看 ROS2 节点和话题

```bash
ros2 node list
ros2 topic list
```

### 检查 RTK

```bash
ros2 topic echo /fix
ros2 topic echo /heading_deg
ls /dev/ttyTHS*
ls /dev/ttyUSB*
```

### 检查雷达

```bash
ip -br addr
ip link show eth0
sudo tcpdump -ni eth0 udp port 6699 or udp port 7788
ros2 topic hz /rslidar_points
```

### 检查超声波

```bash
ls /dev/ttyUSB*
ros2 topic echo /ultrasonic_distances
```

### 检查 CAN

```bash
ip -details link show can0
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up
candump can0
ros2 topic echo /stack_can/feedback
```

---

## 11. 启动前检查通过的判断标准

健康检查通过后，日志中应出现类似信息：

```text
========== DEMO 启动检查通过 ==========
```

随后启动脚本会继续执行：

```bash
ros2 launch main demo.launch.py
```

正式 demo 启动前，应至少确认：

- `/fix` 有有效定位；
- `/heading_deg` 有持续航向角输出；
- `/rslidar_points` 有稳定点云；
- `/ultrasonic_distances` 有至少四路数据；
- `/stack_can/feedback` 有有效 RPM 或油门反馈；
- 急停、遥控接管和人工制动链路可用。

