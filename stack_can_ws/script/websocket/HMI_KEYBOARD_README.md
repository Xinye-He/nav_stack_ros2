# 键盘模拟上位机

此程序独立于 ROS，可在电脑或 Jetson 的交互终端运行，仅需 Python 3.8+ 和 websockets。

```bash
python3 -m pip install websockets
python3 hmi_keyboard_client.py --host 10.144.144.1
```

在 Jetson 本机运行可使用 `--host 127.0.0.1`。请只保留一个9010控制客户端。
默认是仅倾倒/捡拾模式（也可显式加 `--work-only`）。连接后依次发送PAUSE、全零指令、手动接管。
支持 !/9 导航重启、o/i CAN查询恢复，以及j/k、r暂停复位、c/空格清除所有作业位、x急停；速度、转向、RUN和其他功能均屏蔽。
所有cmd都保持速度和转向为0，pick/dump互斥。按r请求解除操作员x触发的急停并保持PAUSE，不自动复位、不绕过任何保护。
PAUSE下VCU是否执行作业需实车协议确认；先在 `/stack_can/status` 查看pick/dump位，不为触发动作开启RUN。

如需恢复完整模拟功能，必须显式指定 `--full-control`。下表描述完整模式，默认不开放这些额外按键。

客户端连接9010后每0.5秒发送应用心跳，同时接收9011状态。相机等待期间仍发送心跳。
仅作业模式启动时发送PAUSE、清零、手动接管；完整模式启动时只发送心跳，按键后发送车辆/相机控制。控制连接断开后不会自动重连或重发指令。

| 按键 | 指令 |
|---|---|
| m / u | 手动接管 / 自动循迹 |
| t | 遥控/循迹模式切换；切到循迹保持暂停，按g启动 |
| g / z | RUN / PAUSE |
| x / r | 急停 / 清零并请求复位 |
| w或4 / 8 / s或1 | 4km/h / 8km/h / 后退4km/h |
| 0 | 速度归零，保留转向 |
| a / d / q / e | -10° / +10° / -20° / +20° |
| c或空格 | 速度、转向归零 |
| j / k / l / h | 倾倒 / 捡拾 / 卸载 / 捡拾动作 |
| f | 任务完成 |
| v / b / p | 开启相机 / 关闭相机 / 查询相机 |
| ? | 显示帮助 |
| Esc或Ctrl+C | 退出 |

速度按键设定持续目标，松键不会自动停车。退出时尝试发送全零手动指令，断线同时触发Jetson原有中止保护。
普通车辆指令无成功回执，屏幕“发送”仅代表发送到连接，需结合状态和车辆反馈判断。
`m`不自动RUN，`g`不自动接管。`r`由服务端执行现有复位逻辑，不绕过围栏等保护。
状态显示“运行故障”可能是锁存的历史故障，按r不等于清除设备总状态。

## 仅查看状态，不接入远控

```bash
python3 hmi_keyboard_client.py --host 10.144.144.1 --status-only
```

只连接9011；连续3秒没有状态会显示断线，随后重连。Ctrl+C退出。

## 离线预览

```bash
python3 hmi_keyboard_client.py --dry-run
python3 hmi_keyboard_client.py --dry-run --keys "jkwsc"
python3 hmi_keyboard_client.py --full-control --dry-run --keys "mgwacxp"
```

只打印将要发送的JSON，不建立连接；`--keys`只允许在离线模式使用。
`--verbose`显示完整9011 JSON与发送的心跳，默认状态变化时才刷新摘要。

## 模拟上传CSV

```bash
python3 hmi_keyboard_client.py --host 10.144.144.1 --upload-fence /path/fence.csv
python3 hmi_keyboard_client.py --host 10.144.144.1 --upload-points /path/points.csv
```

这两种模式只连接9002并等待保存回执，不连接9010。
上传会替换Jetson现有CSV文件；文件格式由服务端校验。

## 测试

`python3 test_hmi_keyboard_client.py` 使用回环地址上的模拟服务，测试映射、心跳、相机回复、状态显示与CSV上传，不连接Jetson车辆服务。

## 保持暂停的复位（仅作业模式r键）

```json
{"type":"reset_estop_paused","request_id":"paused-test-001"}
```

执行节点在同一个回调中替换为全零手动指令、清除允许的操作员急停、设为PAUSE（drive_state=0）。
不会发布RUN，也不会复位循迹状态。回复 `reset_estop_result`，`ok=true` 才代表执行节点确认已复位。
围栏、9010链路、外设/必要控制节点均需有新鲜就绪状态，实体遥控不能正在接管。
保护或断联产生的通用 `/abort` 来源无法确认，本入口拒绝覆盖；原有RUN复位入口不变，但不属于仅作业测试流程。
如回复超时，不能认定已复位。保持PAUSE下机构能否执行取决于VCU，先观察 `/stack_can/status`。
9011设备总状态保留历史运行故障，不会因这条复位回执自动清除；检查回执和实时CAN的estop/drive字段。

此次修改需要重启通信、状态和CAN执行节点后生效；正在运行的旧进程不会热更新。

## n重载、t切换

完整模式n重载成功后选择循迹模式并保持暂停，9010回执message为“路径加载成功，请按键g开始循迹”。t可在启动后直接切换遥控/循迹，无需先n；切到循迹保持暂停，切到遥控清零指令。g在循迹模式下经路径版本、定位、围栏、链路和外设检查后启动。启动时已加载的路径可直接使用；磁盘路径变化后需n重新加载。y已移除。默认work-only屏蔽n/t/g。急停仍须独立处理。

已部署到Jetson；运行中的旧节点需重启导航后加载新逻辑。更新包位于jetson_updates/n_t_control。详见TRAJECTORY_CONTROL_PROTOCOL.md。

## CAN管理（2026-10-30）

默认作业模式和完整模式均支持o查询can0底层状态，i请求单次BUS-OFF恢复。按i表示操作者确认车辆已物理停稳；运行中先按z暂停（默认作业模式已请求暂停）。恢复时维持心跳、屏蔽RUN/复位/路径操作，保留急停，结束保持暂停。接口已恢复但无VCU反馈返回失败，不自动重试。仅在宿主机服务安装且导航加载新版节点后生效，详见CAN_CONTROL_PROTOCOL.md。

## 网络保护和复位更新（2026-10-30）

以执行器实际模式判定：循迹断链仅告警并继续；9010遥控断链急停。重连保持急停，r在默认及完整模式下均为安全暂停复位，不再自动RUN。复位需链路、围栏、外设及控制节点等保护就绪，无其他abort、无CAN维护。等9011显示就绪后再主动选择运行。查询用连接关闭不再触发控制连接断链。详见NETWORK_SAFETY_PROTOCOL.md。

## 9010导航重启（2026-10-30）

首次需在安全停机后手动重启导航加载新接口。以后可使用：

```bash
python3 hmi_keyboard_client.py --host 127.0.0.1 --device-action status
python3 hmi_keyboard_client.py --host 127.0.0.1 --device-action restart --confirm-stopped
```

重启前必须停稳并PAUSE/急停；只读查询不发送控制或心跳。重启请求先返回ACCEPTED，再SCHEDULED，随后连接断开属于预期；须通过9011新boot_id和启动状态确认结果。不自动重试。宿主机安装后的后续启动默认PAUSE，不恢复旧指令缓存。详见DEVICE_RESTART_PROTOCOL.md。

## 导航重启按键（两种键盘模式均可用）

先确认车辆已物理停稳。3秒内连续按两次 `!`：第一次显示确认提示，第二次发送 `{"type":"device","action":"restart","request_id":"restart-唯一ID","confirm_stopped":true}`。其他按键取消第一次确认；超过3秒需重新确认。按 `9` 发送 `device/status` 查询重启任务。

请求发出后禁止继续发送运动命令，保留状态查询和急停；明确拒绝后允许重新确认，结果未知时不自动重试。重启期间9010/9011断开属于预期，需要重开客户端并核对9011新boot_id及启动状态。重启后保持暂停，不自动RUN。

外设启动失败时，只要9010及设备重启接口仍在线即可请求；服务端继续检查停稳和暂停条件。9010未启动时无法通过此按键重启。
