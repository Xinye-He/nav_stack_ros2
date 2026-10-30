# CAN底层状态与安全恢复

9011的`peripherals.can.bus`新增：interface、interface_up、state、link_state、bitrate、tx_error_counter、rx_error_counter、restart_ms、rx_packets、tx_packets、recovering、status_age_ms。约0.5秒采样，超过2秒为CAN_BUS_STATUS_TIMEOUT。data_age_ms仍只表示ROS反馈新鲜度。

底层错误优先列入errors：CAN_INTERFACE_MISSING、CAN_INTERFACE_DOWN、CAN_BUS_OFF、CAN_BUS_STATUS_INVALID、CAN_BUS_STATUS_TIMEOUT。ERROR-WARNING/PASSIVE原样上报，不单独改变反馈有效性；CAN就绪仍需原有效反馈条件。设备总状态保留首次故障，恢复后不会自动复位总状态或启动导航。

## 9010 JSON

查询（ok仅表示查询成功，不表示总线正常）：
```json
{"type":"can","action":"status","request_id":"can-status-001"}
```
```json
{"type":"can_result","action":"status","request_id":"can-status-001","ok":true,"code":"OK","message":"CAN 接口状态查询成功","bus":{"interface":"can0","interface_up":true,"state":"BUS-OFF","bitrate":250000,"tx_error_counter":255,"rx_error_counter":127,"restart_ms":0,"recovering":false}}
```

恢复（操作者须先确认物理停稳，confirm_stopped不能由上位机无条件自动填写）：
```json
{"type":"can","action":"recover","request_id":"can-recover-001","confirm_stopped":true}
```

RUN时先PAUSE；有效地速或轮速超过0.05m/s、实体遥控接管、无法确认执行器维护暂停时拒绝。只允许启动失败且从未就绪、图中无执行器的情况走无执行器恢复。BUS-OFF时不能以失效反馈的默认0证明停止，所以必须由操作者确认。

流程：执行器确认暂停维护（或确认启动失败且无执行器）→宿主机单次恢复can0→等待最多5秒的新有效VCU反馈→再次查询总线状态→返回并保持暂停。恢复期间屏蔽RUN、路径操作和复位，急停始终可用。维持原急停锁存，不自动RUN，不自动重启导航。只恢复BUS-OFF，不改波特率，不执行down/up或设置自动恢复。接口已ERROR-ACTIVE时不重复复位，仍验证反馈。

成功：
```json
{"type":"can_result","action":"recover","request_id":"can-recover-001","ok":true,"code":"OK","message":"can0 已恢复，已收到有效 VCU 反馈；车辆保持暂停，未解除急停","interface_recovered":true,"feedback_valid":true,"bus":{"interface":"can0","state":"ERROR-ACTIVE","interface_up":true}}
```

接口恢复但反馈未恢复：
```json
{"type":"can_result","action":"recover","request_id":"can-recover-001","ok":false,"code":"CAN_FEEDBACK_NOT_RECOVERED","message":"接口已恢复，但未收到有效 VCU 反馈，请检查供电、接线和波特率","interface_recovered":true,"feedback_valid":false,"bus":{"interface":"can0","state":"ERROR-ACTIVE","interface_up":true}}
```

| code | 含义 |
|---|---|
| OK | 查询成功或完整恢复成功 |
| INVALID_REQUEST | 操作或request_id不合法 |
| CAN_STOP_CONFIRMATION_REQUIRED | 缺少操作者停止确认 |
| CAN_SAFETY_REJECTED | 未暂停、移动中、实体遥控接管或维护确认失败 |
| CAN_BUSY | 恢复进行中，或恢复期间请求控制/路径操作 |
| CAN_SERVICE_UNAVAILABLE | 宿主机服务不可达/超时，结果可能未知，不自动重试 |
| CAN_INTERFACE_MISSING / CAN_INTERFACE_DOWN | 接口不存在/未启用 |
| CAN_NOT_BUS_OFF | 非BUS-OFF且非ERROR-ACTIVE，不复位 |
| CAN_RECOVERY_FAILED | ip命令失败/超时 |
| CAN_INTERFACE_NOT_RECOVERED | 接口未恢复或再次出错 |
| CAN_FEEDBACK_NOT_RECOVERED | 接口恢复但有效反馈未恢复 |

每个进程缓存最近100个恢复结果，重复request_id不重复操作；进程重启缓存会清空。心跳每0.5秒继续发送，不自动重试恢复。

## 客户端

```bash
python3 hmi_keyboard_client.py --host 127.0.0.1
```

- o查询CAN；i确认车辆已停止并请求恢复；默认作业模式与完整模式均支持。
- 9011只读仍使用`--status-only --verbose`，ROS显示节点也展示总线状态。
- 使用已有9010控制连接；原有断联保护仍然有效，不要为了查询另外开关控制连接。

## 服务与生效

宿主机安装命令：
```bash
sudo bash /home/xinye30/nav_stack_ros2/stack_can_ws/script/can_control/install_service.sh
```

安装nav-can-control.service并开机启动，Unix socket位于script/can_control/.runtime/control.sock，group docker，660。服务只拥有CAP_NET_ADMIN，执行固定can0的查询和单次恢复，不接受任意shell或波特率参数。

编译main后，安全停机再重启nav-stack-ros2.service，让9010、9011和执行器加载新逻辑；不要单独杀状态服务。若原来startup_failed，恢复CAN后仍需按原启动流程重新启动控制程序，不自动RUN。
