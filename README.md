# 农田草捆捡拾自主作业无人车

本仓库是一个基于 **ROS2 + Docker + Jetson Orin NX** 的农业无人车软件工程，面向农田草捆捡拾场景，支持 RTK 路径跟踪、激光雷达草捆检测、草捆末端对准、VCU/CAN 底盘控制、超声波状态检测、电子围栏、电机堵转保护和启动/运行期健康检查。

---

## 项目目标

项目名称：**《农田草捆捡拾自主作业无人车》**

系统目标是在已规划作业路径下，实现农业无人车的完整自动作业流程：

1. 通过 RTK/GNSS 获取车辆位置和航向；
2. 根据 CSV 路径文件进行自主循迹行驶；
3. 在草捆任务点附近使用激光雷达识别并定位草捆；
4. 根据草捆相对距离和角度完成车辆对准、靠近和拾取动作；
5. 通过 CAN 与 VCU 通信，控制底盘速度、转向、急停、拾取和卸货动作；
6. 通过健康检查、电机保护、电子围栏和急停锁存提升系统安全性；
7. 使用 Docker 统一 Jetson 端运行环境，降低部署成本。

---

## 当前实现状态

当前仓库已经接入并重点维护的内容：

- Docker 一键启动脚本：`docker/demo.sh`
- 基础传感器启动：`docker_startup.launch.py`
- 正式自动作业启动：`demo.launch.py`
- 启动前健康检查：`script/start_demo_after_health_check.sh`
- 运行期健康监控：`script/health_monitor.py`
- RTK/NMEA 输入与车辆中心点转换
- CSV 路径跟踪与任务点等待
- RoboSense E1R 激光雷达点云输入
- 基于点云几何特征的草捆检测：`bale_detector.cpp`
- 草捆对准与拾取控制：`bale_align_controller.py`
- CAN 控制命令裁决：`stack_can_executor.py`
- CAN 反馈读取：`can_feedback_node.py`
- 电机堵转保护：`motor_protection_node.py`
- 电子围栏：`geofence_monitor.py`
- 一拖四超声波距离读取：`ultrasonic.py`
- WebSocket 上位机/调试接口

项目初期规划包含视觉网络与激光雷达协同识别草捆，Jetson 端环境也保留了视觉模型训练和推理相关依赖。当前代码闭环中，正式参与自动作业的是 RTK、激光雷达、CAN、超声波和安全监控链路。视觉目标检测可作为后续增强模块继续接入。

---

## 硬件平台

| 模块 | 型号 / 说明 |
|---|---|
| 计算平台 | NVIDIA Jetson Orin NX |
| RTK/GNSS | 维特智能 WTRTK-982 |
| 激光雷达 | RoboSense E1R |
| 超声波 | 一拖四超声波测距模块 |
| 底盘控制 | VCU，通过 SocketCAN 通信 |
| CAN 协议 | 与 VCU 供应商协商拟定 |
| 软件框架 | ROS2 + Docker |

---

## 软件架构

```text
nav_stack_ros2/
├── docker/
│   └── demo.sh                         # Jetson 端 Docker 启动脚本
├── stack_can_ws/
│   ├── src/
│   │   ├── main/                       # 主控制功能包
│   │   │   ├── launch/
│   │   │   │   ├── docker_startup.launch.py
│   │   │   │   └── demo.launch.py
│   │   │   ├── config/
│   │   │   │   └── params.yaml
│   │   │   └── main/
│   │   │       ├── traj_waypoint_follower.py
│   │   │       ├── stack_can_executor.py
│   │   │       ├── can_feedback_node.py
│   │   │       ├── motor_protection_node.py
│   │   │       ├── bale_align_controller.py
│   │   │       ├── geofence_monitor.py
│   │   │       └── ultrasonic.py
│   │   ├── robosense_driver/           # RoboSense 雷达驱动与草捆点云检测
│   │   ├── nmea_bridge/                # RTK/NMEA 解析
│   │   └── stack_msgs/                 # 自定义 ROS2 消息
│   ├── script/
│   │   ├── health_monitor.py
│   │   ├── start_demo_after_health_check.sh
│   │   └── server_all_ws.py
│   └── data/
│       ├── points.csv                  # 作业路径点
│       └── fence.csv                   # 电子围栏点
```

---

## 主要 ROS2 节点

| 节点 | 作用 |
|---|---|
| `nmea_bridge_node` | 读取 RTK/NMEA 数据，发布 GNSS 定位和航向相关数据 |
| `rtk_center_from_nmea` | 根据 RTK 安装偏移，将主天线位置转换为车辆中心点位置 |
| `traj_waypoint_follower` | 读取 CSV 路径，进行路径跟踪、任务点等待和卸货点处理 |
| `rsview` | RoboSense E1R 雷达驱动，发布 `/rslidar_points` |
| `bale_detector` | 基于激光雷达点云检测草捆目标，发布 `/bale_target` |
| `bale_align_controller` | 在任务点根据草捆距离和角度控制车辆对准、靠近和拾取 |
| `stack_can_executor` | 对路径、遥控、草捆命令进行安全裁决，并发送 CAN 控制帧 |
| `can_feedback_node` | 读取 VCU/CAN 电机反馈，发布车速、转速、油门等状态 |
| `motor_protection_node` | 检测高油门低转速堵转风险，触发 `/abort` |
| `geofence_monitor` | 根据 `fence.csv` 判断是否越界，必要时触发 `/abort` |
| `ultrasonic` | 读取四路超声波距离，发布近距离障碍状态 |
| `health_monitor.py` | 启动前和运行期检查关键传感器与 CAN 状态 |

---

## 数据流

```text
RTK / NMEA
   ↓
nmea_bridge_node
   ↓
/fix, /heading_deg
   ↓
rtk_center_from_nmea
   ↓
/fix_center, /vehicle_heading_deg
   ↓
traj_waypoint_follower
   ↓
/stack_cmd/traj

RoboSense E1R
   ↓
/rslidar_points
   ↓
bale_detector
   ↓
/bale_target
   ↓
bale_align_controller
   ↓
/stack_cmd/bale

/stack_cmd/traj     /stack_cmd/bale     /stack_cmd/teleop
        \                |                /
         \               |               /
             stack_can_executor
                    ↓
              SocketCAN / can0
                    ↓
                   VCU
```

安全链路：

```text
health_monitor / motor_protection_node / geofence_monitor / teleop
                       ↓
                    /abort
                       ↓
              stack_can_executor
                       ↓
                 ESTOP latch
                       ↓
             CAN stop / estop frame
```

---

## 草捆检测说明

当前 `bale_detector.cpp` 使用激光雷达点云进行草捆检测。核心流程如下：

1. 订阅 `/rslidar_points`；
2. 去除 NaN 点；
3. 根据雷达安装高度、俯仰角和横滚角建立地面坐标系；
4. 只保留车辆前方、指定距离范围内的点；
5. 将雷达点云转换到 ground 坐标系；
6. 按离地高度筛选草捆主体点；
7. 将点云投影到地面 XY 平面；
8. 使用 PCL 欧式聚类提取候选簇；
9. 对每个簇进行 2D PCA，计算长边、短边、高度厚度、长宽比和占据率；
10. 根据草捆几何尺寸筛选候选目标；
11. 选择距离最近的候选草捆；
12. 发布 `/bale_target`。

调试输出：

| 话题 | 说明 |
|---|---|
| `ground_aligned_points` | 转换到地面坐标系后的点云 |
| `height_filtered_points` | 高度筛选后的点云 |
| `projected_cluster_points` | 投影聚类点云 |
| `bale_markers` | 草捆候选框和中心点 |
| `/bale_target` | 草捆距离、角度和有效标志 |

`/bale_target` 字段含义：

| 字段 | 说明 |
|---|---|
| `distance_m` | 草捆相对车辆的平面距离 |
| `angle_deg` | 草捆相对车辆前方的水平角度，当前工程约定左负右正 |
| `valid` | 是否检测到有效草捆 |

---

## 启动方式

### 1. 克隆仓库

```bash
git clone --recursive https://github.com/Xinye-He/nav_stack_ros2.git
cd nav_stack_ros2
```

### 2. 启动 Docker demo

```bash
cd ~/nav_stack_ros2
./docker/demo.sh
```

`docker/demo.sh` 会：

1. 删除旧的 `nav_stack_ros2` 容器；
2. 使用 `xinye30/ucar-jetson:nav-stack-ros2` 镜像启动新容器；
3. 挂载 `/dev`、Jetson Argus socket、工作空间和显示环境；
4. 使用 host 网络和 NVIDIA runtime；
5. source ROS 环境和工作空间；
6. 后台启动 `docker_startup.launch.py`；
7. 执行启动前健康检查；
8. 健康检查通过后启动 `demo.launch.py`；
9. 命令结束后保留容器 shell，便于查看日志和排障。

### 3. 手动启动基础节点
当启动前健康检查失败，即外设未正常运行时，需要手动运行基础节点脚本

```bash
ros2 launch main docker_startup.launch.py
```

基础节点包括：

- RTK/NMEA 输入；
- WebSocket 服务；
- CAN 反馈；
- 超声波；
- RTK 中心点转换；
- RoboSense 雷达驱动。

### 4. 手动启动正式 demo

```bash
ros2 launch main demo.launch.py
```

正式 demo 包括：

- 运行期健康监控；
- CAN 反馈；
- 电机保护；
- DR/里程计估计；
- 路径跟踪；
- CAN 执行器；
- 电子围栏；
- 草捆检测；
- 草捆对准控制。

---

## 启动前健康检查

默认启动流程不会直接启动正式 demo，而是先运行：

```bash
/root/stack_can_ws/script/start_demo_after_health_check.sh
```

该脚本检查以下话题：

| 检查项 | 话题 |
|---|---|
| RTK/GPS | `/fix` |
| 航向角 | `/heading_deg` |
| 激光雷达 | `/rslidar_points` |
| 超声波 | `/ultrasonic_distances` |
| CAN 反馈 | `/stack_can/feedback` |

如果检查失败，`demo.launch.py` 不会启动。日志位置：

```bash
cat /tmp/health_monitor_startup.log
cat /tmp/docker_startup.log
```

修复硬件或配置后，可在容器内重新执行：

```bash
/root/stack_can_ws/script/start_demo_after_health_check.sh
```

---

## 关键配置

主配置文件：

```text
stack_can_ws/src/main/config/params.yaml
```

常用路径：

| 文件 | 说明 |
|---|---|
| `/root/stack_can_ws/data/points.csv` | 自动作业路径点 |
| `/root/stack_can_ws/data/fence.csv` | 电子围栏多边形点 |

常用参数：

| 参数组 | 说明 |
|---|---|
| `traj_waypoint_follower` | 路径跟踪、任务点、速度/转角档位 |
| `stack_can_executor` | CAN 接口、控制帧 ID、速度编码 |
| `can_feedback_node` | 电机反馈 CAN ID、轮速换算、反馈话题 |
| `motor_protection_node` | 高油门低转速堵转检测阈值 |
| `geofence_monitor` | 围栏 CSV、警告距离、越界急停 |
| `bale_align_controller` | 草捆对准阈值、转向周期、靠近距离、拾取保持时间 |
| `bale_detector` | 点云筛选、聚类和草捆几何判据 |

---

## 常用调试命令

查看话题：

```bash
ros2 topic list
```

查看 RTK：

```bash
ros2 topic echo /fix
ros2 topic echo /heading_deg
ros2 topic echo /fix_center
ros2 topic echo /vehicle_heading_deg
```

查看雷达和草捆检测：

```bash
ros2 topic hz /rslidar_points
ros2 topic echo /bale_target
ros2 topic echo /projected_cluster_points
```

查看 CAN：

```bash
ip link show can0
ros2 topic echo /stack_can/feedback
ros2 topic echo /stack_can/status
```

查看安全状态：

```bash
ros2 topic echo /abort
ros2 topic echo /geofence_state
ros2 topic echo /geofence_ok
ros2 topic echo /ultrasonic_obstacle_near
```

查看日志：

```bash
cat /tmp/health_monitor_startup.log
cat /tmp/docker_startup.log
```

---

## rosbag 测试

播放已录制点云：

```bash
ros2 bag play <bag_dir> --topics /rslidar_points
```

记录点云：

```bash
ros2 bag record /rslidar_points -o <filename>
```

记录完整调试数据：

```bash
ros2 bag record \
  /fix \
  /heading_deg \
  /fix_center \
  /vehicle_heading_deg \
  /rslidar_points \
  /bale_target \
  /stack_cmd/traj \
  /stack_cmd/bale \
  /stack_can/feedback \
  /stack_can/status \
  /abort \
  -o debug_run
```

---

## 安全机制

### 1. 启动前健康检查

正式 demo 启动前检查 RTK、航向、雷达、超声波和 CAN 反馈。任一关键输入缺失时，不启动自动作业。

### 2. 运行期健康监控

`demo.launch.py` 中启动 `health_monitor.py`，运行中持续检查关键话题。如果 RTK、航向、雷达、超声波或 CAN 长时间无数据，主动终止 launch。

### 3. 急停锁存

`stack_can_executor` 对 `/abort` 和 `/drive_cmd` 急停状态进行锁存。急停后持续输出停车和 ESTOP CAN 帧。

### 4. 电机堵转保护

`motor_protection_node` 检测左右电机是否出现“高油门 + 低转速 + CAN 控制模式”的状态。持续超过阈值后发布 `/abort=True`。

### 5. 电子围栏

`geofence_monitor` 根据 `fence.csv` 判断车辆是否位于作业区域内。越界时可触发 `/abort=True`。

### 6. 遥控接管

`stack_can_executor` 支持遥控接管状态。安全裁决优先级高于自主路径和草捆对准命令。

---

## 当前边界

请注意当前仓库的真实边界：

1. 当前自动作业主链路以 RTK、激光雷达、CAN 和 ROS2 控制为主；
2. 草捆检测当前代码主要基于激光雷达点云几何特征；
3. 视觉目标检测属于系统规划和可扩展方向，尚未在当前分支中作为正式 ROS2 闭环节点接入；
4. 当前作业路径依赖 `points.csv`，草捆任务点需要提前规划或采集；
5. 草捆检测参数与雷达安装高度、俯仰角、点云密度、草捆尺寸和现场环境强相关；
6. 电机堵转保护、电子围栏和超声波阈值需要结合实车继续标定；
7. 不建议在未确认 RTK、雷达、CAN、超声波和急停链路正常前进行自动作业测试。

---

## 后续计划

- 接入视觉草捆检测 ROS2 节点；
- 将视觉检测框与激光雷达点云进行关联；
- 使用视觉结果降低雷达几何规则误检；
- 完善超声波近距离障碍到 `/abort` 的控制链路；
- 增加自动 rosbag 记录和异常日志归档；
- 增加 Web 状态面板，显示 RTK、雷达、CAN、安全状态和草捆目标；
- 对草捆检测、路径跟踪、拾取成功率进行系统化实测统计。

---

## 免责声明

本项目面向实车农业无人车测试环境，涉及底盘控制、CAN 通信和急停逻辑。请在空旷、安全、可人工接管的环境中测试，确认急停链路和遥控接管有效后再进行自动作业。

机器不会替你承担现场事故责任，虽然它可能比人类更早发现你忘了开 RTK。
