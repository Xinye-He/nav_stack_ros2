# 按执行模式处理9010断链

## 行为

| 执行器实际选择 | 9010控制连接断开/心跳超时 |
|---|---|
| 自动循迹 | 仅网络警告，继续循迹，不清零循迹指令、不发布abort |
| 9010遥控（含已暂停的遥控选择） | 网络急停锁存，输出停止；循迹节点暂停 |
| 本地键盘/实体遥控 | 网络仅告警，不替代其自身安全逻辑 |
| 已有其他急停 | 保留原保护，网络事件不覆盖原原因 |
| 运行中控制模式无法明确判断 | 保守急停 |

判断在stack_can_executor中完成，不相信上位机自报模式或桥接节点缓存。网络保护用专用/safety/network_event与/safety/network_result，不与围栏/电机保护共享的/abort混淆。执行器未在2秒内确认处置时仍采取保守通用急停，此异常不能按已确认的纯网络急停自动解除。

控制连接的有效数据/心跳超时默认1.5秒。推荐心跳0.5秒。仅有查询的连接不成为控制连接，其关闭不触发断链保护；其他查询客户端也不能掩盖实际控制连接的心跳丢失。上位机正常控制消息或首个心跳建立控制归属，重连后心跳可重新取得归属。

## 重连和人工复位

重连仅恢复链路，不自动解除急停或RUN。
默认/完整模式的r统一发送：
```json
{"type":"reset_estop_paused","request_id":"reset-network-001"}
```
旧的9010 `{"type":"reset_estop"}` 也映射到同一安全暂停复位；旧的本地桥接r同样不再自动RUN。

复位要求：明确的网络急停或普通操作急停、无其他/abort、无实体遥控接管、围栏/链路/外设及控制节点保护全部为true且最近2秒内更新、无CAN维护。网络故障恢复但围栏仍未满足要求时不会复位。

成功回执示例：
```json
{"type":"reset_estop_result","request_id":"reset-network-001","ok":true,"message":"急停已解除，保持PAUSE、零速零转向","drive_state":0,"estop":false,"cleared_reason":"network_disconnected","cleared_network_event_id":"network-unique-event-id"}
```

心跳超时原因是network_heartbeat_timeout，连接断开是network_disconnected。通用/abort若同时触发则覆盖为abort并阻止按纯网络原因解除。原直接ROS /reset_estop旧RUN复位不能覆盖网络锁存，请使用上述安全复位请求。

复位成功后等待9011重新检查就绪，再主动切换/启动。r不再RUN；n重载后选择零速遥控，t切换并按原规则启动循迹。

## 9011 与历史故障

新增network_status：
```json
{"network_status":{"state":"disconnected","severity":"warning","message":"循迹模式上位机断链，仅告警，循迹继续运行","last_event":{"type":"network_event","action":"warning","reason_code":"network_disconnected"}}}
```

state为waiting_connection/connected/disconnected/unknown；severity为none/warning/emergency。

循迹告警不将设备从ready改为runtime_fault，vehicle_state仍可为trajectory。遥控网络急停会成为runtime_fault，原因有明确network_*编码。

只有手动暂停复位成功、执行器反馈确认同一复位请求、当前仍暂停且安全就绪、链路正常、外设/控制节点/CAN无故障时，才清除对应类别的历史网络故障；不改变boot_id，诊断历史保留，并记录acknowledged_fault。围栏、电机、CAN、启动失败及旧版来源不明的abort不会被此操作清除。重复网络故障后，可用当前成功的人工网络复位确认清除已保留的网络历史；旧请求回执不能清除新故障。

ROS终端和键盘摘要会显示network_status的网络警告；详细9011 JSON提供last_event。当前显示节点会将network_*原因翻译为明确中文，不重复另列急停原因。

## 生效

修改需编译main，并在车辆安全停机后重启nav-stack-ros2.service，让执行器、桥接、状态服务同时加载新版。部署过程不进行真实断链急停、不发送车辆指令、不重启导航。
