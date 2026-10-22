# 上位机相机控制

使用现有 `ws://10.244.241.213:9010` 长连接，发送 JSON。相机命令和车辆命令使用同一连接。
上位机继续发送应用层心跳（建议每 0.5 秒 `{ "type": "heartbeat" }`），包括相机启动期间。
不要为相机另建一个短连接；9010 的断线中止保护仍生效。

```json
{"type":"camera","action":"start","request_id":"camera-start-001"}
```

- `start`：执行 Jetson 宿主机 `script/video_usb/start_monitor_new.sh`。
- `stop`：执行同目录 `stop_monitor.sh`。
- `status`：查询 MediaMTX、各相机进程和 RTSP 流是否就绪。
- `request_id`：推荐提供 1~128 字符唯一标识；省略时由 Jetson 自动生成。
- 命令异步执行，不阻塞车辆消息和心跳。同一时刻只执行一个相机操作，其余返回 `CAMERA_BUSY`。
- 同一 `request_id` 的重复启停返回缓存结果，避免重复重启；新操作使用新 ID。

脚本完成后在原 9010 连接返回结果，`request_id` 对应请求，例如：

```json
{
  "type":"camera_result",
  "request_id":"camera-start-001",
  "action":"start",
  "ok":true,
  "code":"OK",
  "message":"相机已开启",
  "exit_code":0,
  "output":"脚本输出（最多末尾4000字符）",
  "camera_state":"running",
  "mediamtx_running":true,
  "cameras":[
    {"name":"cam0","device":"/dev/video0","process_running":true,"stream_ready":true,"url":"rtsp://10.244.241.213:8554/cam0"},
    {"name":"cam1","device":"/dev/v4l/by-id/usb-camera-video-index0","process_running":true,"stream_ready":true,"url":"rtsp://10.244.241.213:8554/cam1"}
  ]
}
```

失败时 `ok=false`；应显示 `message`，根据 `code` 判断错误。不是所有失败都包含相机状态或退出码。
`CAMERA_DEVICE_MISSING` 设备不存在、`CAMERA_CONFIG_MISSING` 配置缺失、`CAMERA_SCRIPT_FAILED` 执行失败或未达到目标状态、
`CAMERA_SERVICE_UNAVAILABLE` 宿主机服务不可用、`CAMERA_BUSY` 正在执行其他操作、`INVALID_REQUEST` 参数错误。
`camera_state`：`running` 两路均可 RTSP DESCRIBE；`stopped` 无推流进程且 MediaMTX 未运行；`partial` 部分启动。
查询成功 `ok=true` 仅代表查询完成，仍须检查 `camera_state`。

## 执行环境

`docker/demo.sh` 在创建导航容器前运行 `ensure_camera_service.sh`，只启动宿主机控制服务，不自动打开相机。
容器经工作空间内的 Unix socket `.runtime/control.sock` 转交白名单操作，宿主机使用 xinye30 权限执行两个固定脚本。
无需向导航容器挂载 Docker socket。宿主机服务日志在 `script/video_usb/.runtime/service.log`。
原启动脚本依然使用 mediamtx 容器名和 cam*.pid 文件，确保不同时运行其他脚本争用同一推流资源。

已补齐 `config.sh`、`mediamtx.yml`。cam0 使用检测到的 IMX219 CSI 相机（/dev/video0，nvarguscamerasrc），cam1 使用 USB/UVC 相机（优先按 /dev/v4l/by-id 自动发现）。
当前 USB 相机尚未枚举时，会跳过 USB 并尝试启动 CSI；连接 USB 相机后无需重启宿主机代理，下一次命令会重新读取配置。
USB 分支沿用原 MJPEG 1920x1080@30 配置，若相机不支持，需根据实际格式修改 config.sh/管线。

如需单独启动宿主机代理：

```bash
bash /home/xinye30/nav_stack_ros2/stack_can_ws/script/video_usb/ensure_camera_service.sh
```

修改 main 包后应编译，并重新启动通信程序；不会对已运行的旧进程热更新。

## 按可用相机启动

start 会跳过未识别设备。有任一路 RTSP 流就绪即允许部分成功：
- 全部就绪：ok=true，code=OK，camera_state=running。
- 部分就绪：ok=true，code=PARTIAL_READY，camera_state=partial；检查 cameras 每路的 device_present 和 stream_ready。
- 没有可用设备：ok=false，code=CAMERA_DEVICE_MISSING。
- 启动后没有任何流就绪或脚本失败：ok=false，code=CAMERA_SCRIPT_FAILED。

重复开启已就绪的可用相机不会重启推流；USB接入后再次start（使用新request_id）会重新读取配置并尝试启动两路。
stop 仍停止所有由该脚本管理的流。status 的 ok=true 只表示查询成功，仍以camera_state及各路stream_ready判断就绪。
