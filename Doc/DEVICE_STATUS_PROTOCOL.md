# 上位机设备状态协议 v1

## 连接和刷新

连接 `ws://10.244.241.213:9011/status`。只接收状态，不要向此端口发送控制指令。
9010 仍用于远控。连接后立即收到完整 JSON；每秒至少发送一次，状态变化时提前发送。
上位机首次连接前显示“等待连接”；断开或连续 3 秒未收消息显示“断线”，外设置灰，清空车辆模式。
重连后使用新快照覆盖旧状态。`boot_id` 改变表示新启动；`seq` 是本次服务进程内递增序号。
`timestamp` 使用 UTC ISO 8601，上位机可转换成本地时间。通信超时使用接收端计时。

## 数据示例

```json
{
  "type": "device_status",
  "version": 1,
  "boot_id": "a-unique-boot-id",
  "seq": 20,
  "timestamp": "2026-10-21T15:00:00+00:00",
  "device_state": "ready",
  "device_message": "",
  "vehicle_state": "trajectory",
  "peripherals": {
    "rtk": {"state": "ready", "errors": [], "data_age_ms": 80},
    "lidar": {"state": "ready", "errors": [], "data_age_ms": 90},
    "ultrasonic": {"state": "ready", "errors": [], "data_age_ms": 30},
    "can": {"state": "ready", "errors": [], "data_age_ms": 40}
  }
}
```

## 状态枚举

设备：`starting` 启动中、`startup_failed` 启动失败、`ready` 就绪、`runtime_fault` 运行故障。
`waiting_connection` 等待连接、`disconnected` 断线由上位机判断，不由 Jetson 发送。

外设：`starting` 启动中、`ready` 就绪、`error` 错误。错误时显示 `errors[].message`。
`errors` 可同时包含多个错误；`code` 用于程序判断，`message` 用于显示。驱动原始报错可能带 `source`。
`data_age_ms` 是距最近有效数据的时间；RTK 取定位/航向中较旧的一项；未收到完整数据则为 null。

车辆：`trajectory` 循迹中、`remote` 9010 遥控模式、`null` 暂不显示。
只有设备就绪、执行节点反馈新鲜、安全状态正常、处于 RUN 时才显示模式。
连接 9010 不等于接管。`remote` 还要求执行节点正在采用 `/stack_cmd/websocket` 指令，且 9010 链路有效。
本地键盘、实体遥控、草捆对准、暂停、急停均不伪装成这两种模式，返回 null。

## 外设判断与错误码

- RTK：定位状态非负、经纬度有限且在合法范围，并且航向有限。此标准不是 RTK 固定解。
  `RTK_FIX_TIMEOUT`、`RTK_FIX_INVALID`、`RTK_HEADING_TIMEOUT`、`RTK_HEADING_INVALID`。
- 雷达：点云点数和载荷非空。`LIDAR_TIMEOUT`、`LIDAR_INVALID`。
- 超声波：至少四路有限数值；0 仍允许，保持无回波处理。`ULTRASONIC_TIMEOUT`、`ULTRASONIC_INVALID`。
- CAN：持续收到反馈，至少一个电机 RPM/油门有效位为真。`CAN_TIMEOUT`、`CAN_INVALID`。
- `/rosout` 中 RTK、雷达、超声波、CAN 驱动的 ERROR/FATAL 附加为 `DRIVER_ERROR`；不猜测硬件根因。
  仅写终端且未进入 `/rosout` 的错误不会包含原始文本，但仍显示无数据/超时现象。
- CAN 执行接口不可用、发送失败、执行状态超时、必需控制节点退出、急停/中止，也会写入设备总错误。

## 启动和故障恢复

`docker/demo.sh` 加载环境后，先启动独立 `status_service.py`，再启动外设和健康检查。
健康检查和状态服务共用数据有效性规则，5 秒启动窗口、2 秒运行数据时效默认值。
外设检查成功后进入控制启动阶段；节点可见、执行状态新鲜且 CAN 可用后才显示就绪。
控制启动最多等待 15 秒，超时显示启动失败。
运行故障和启动失败均保留首个错误，不因外设短暂恢复自动恢复总状态。
外设卡片继续实时更新；修复原因后，重新执行 `script/start_demo_after_health_check.sh` 进行检查和启动。
重试前须确认旧控制启动进程已退出。这里不增加车辆自动恢复行驶行为。
状态服务不发布车辆运动指令；既有 health_monitor 仍负责外设故障时终止 demo。

## 部署

修改过 `main` 包，需要在导航容器加载 ROS 环境后编译：

```bash
cd /root/stack_can_ws
colcon build --packages-select main
source install/setup.bash
```

随后按原方式启动 `docker/demo.sh`。旧 `remote_bridge` 占用 9001/9003/9010 时必须先停止，否则通信节点启动失败。
`/tmp/device_status.log` 为状态服务日志；`/tmp/nav_device_status.json` 为启动生命周期状态。
监控服务从容器环境就绪后开始；Docker 或 ROS 环境自身启动失败，上位机只能显示未连接。

测试（ROS 和已编译 main、stack_msgs 环境）：

```bash
python3 script/tests/test_device_status.py
```

测试只注入状态和在测试端口建立状态连接，不向车辆 CAN 发送控制。
