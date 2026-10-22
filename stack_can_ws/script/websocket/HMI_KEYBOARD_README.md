# 键盘模拟上位机

此程序独立于 ROS，可在电脑或 Jetson 的交互终端运行，仅需 Python 3.8+ 和 websockets。

```bash
python3 -m pip install websockets
python3 hmi_keyboard_client.py --host 10.244.241.213
```

在 Jetson 本机运行可使用 `--host 127.0.0.1`。请只保留一个9010控制客户端。
默认是仅倾倒/捡拾模式（也可显式加 `--work-only`）。连接后依次发送PAUSE、全零指令、手动接管。
只允许j/k、r暂停复位、c/空格清除所有作业位、x急停；速度、转向、RUN和其他功能均屏蔽。
所有cmd都保持速度和转向为0，pick/dump互斥。按r请求解除操作员x触发的急停并保持PAUSE，不自动复位、不绕过任何保护。
PAUSE下VCU是否执行作业需实车协议确认；先在 `/stack_can/status` 查看pick/dump位，不为触发动作开启RUN。

如需恢复完整模拟功能，必须显式指定 `--full-control`。下表描述完整模式，默认不开放这些额外按键。

客户端连接9010后每0.5秒发送应用心跳，同时接收9011状态。相机等待期间仍发送心跳。
仅作业模式启动时发送PAUSE、清零、手动接管；完整模式启动时只发送心跳，按键后发送车辆/相机控制。控制连接断开后不会自动重连或重发指令。

| 按键 | 指令 |
|---|---|
| m / u | 手动接管 / 自动循迹 |
| t | 由服务端切换手动/自动 |
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
python3 hmi_keyboard_client.py --host 10.244.241.213 --status-only
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
python3 hmi_keyboard_client.py --host 10.244.241.213 --upload-fence /path/fence.csv
python3 hmi_keyboard_client.py --host 10.244.241.213 --upload-points /path/points.csv
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
