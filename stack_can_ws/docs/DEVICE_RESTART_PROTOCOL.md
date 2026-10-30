# 9010远程重启导航

## 请求

```json
{"type":"device","action":"restart","request_id":"restart-001","confirm_stopped":true}
```

confirm_stopped是操作者确认车辆已物理停稳，不能用失效CAN反馈中的默认0自动代填。RUN或有效轮速显示仍在运动时拒绝；先暂停/急停。与CAN恢复共用执行器暂停维护互锁，不允许同时维护。控制节点不存在时仅支持本次启动失败且从未就绪的已确认停止场景。

## 两阶段回执

先准备，成功发送第一条WS回执后才提交：
```json
{"type":"device_result","action":"restart","request_id":"restart-001","ok":true,"code":"RESTART_ACCEPTED","message":"已接受重启请求，随后9010/9011将断开；请重连并通过新boot_id检查启动状态","startup_mode":"paused","boot_id_before":"原9011启动ID"}
```

提交前再次确认暂停，宿主机提交后发第二条：
```json
{"type":"device_result","action":"restart","request_id":"restart-001","ok":true,"code":"RESTART_SCHEDULED","message":"已安排导航重启，启动后保持暂停","service":"nav-stack-ros2.service","job":{"request_id":"restart-001","state":"scheduled","restart_after_sec":2.0,"startup_mode":"paused","boot_id_before":"原9011启动ID"}}
```

宿主机管理服务独立于导航运行，延迟2秒只执行固定的`/bin/systemctl restart nav-stack-ros2.service`。导航重启会断开9010和9011，操作者需重连并检查boot_id改变、starting到ready。ACCEPTED/SCHEDULED或systemctl返回成功均不证明车辆导航已就绪。

若第一条WS回执无法发送，则不提交，准备请求被取消/过期。提交回执丢失则结果未知，保持暂停，不自动重试。重复同一request_id不会再次安排重启。准备请求15秒过期；每个宿主机进程保存最近100个任务，并持久化；管理服务重启不会重放未完成命令。

## 查询任务

```json
{"type":"device","action":"status","request_id":"restart-status-001"}
```

回执含service、startup_pause_armed、job。job.state为prepared/scheduled/restarting/completed/failed/cancelled/expired/unknown。查询无控制、无重启动作，不建立控制连接归属。

| code | 含义 |
|---|---|
| OK | 查询成功 |
| INVALID_REQUEST | 操作、ID或内部令牌不正确 |
| DEVICE_STOP_CONFIRMATION_REQUIRED | 缺少停稳确认 |
| DEVICE_BUSY | 设备/CAN维护或未完成重启任务存在 |
| DEVICE_REQUEST_REJECTED / DEVICE_SAFETY_REJECTED | 停止维护检查未通过、服务不可用或提交证明过期；读取message |
| RESTART_ACCEPTED | 第一阶段接受，尚不代表已执行 |
| RESTART_SCHEDULED | 已提交，延迟执行 |
| RESTART_ALREADY_REQUESTED | 重复请求，读取job.state，不重新执行 |
| RESTART_FAILED | 任务确定失败或未执行，保持暂停 |
| RESTART_RESULT_UNKNOWN / RESTART_VERIFY_REQUIRED | 无法确认结果，保持维护暂停，需人工查询9011新boot_id和宿主机 |

结果未知时不能自动重发新ID，避免重启循环；必要时通过SSH检查systemd状态。维护期间仅查询、心跳和急停可用。不自动解除既有急停，也不自动RUN。

## 启动安全

宿主机服务安装时即创建持久的`script/device_control/.runtime/start_paused.json`。docker/demo.sh据此强制向容器传入NAV_START_PAUSED=1，使桥接节点启动广播PAUSE；执行器和循迹节点本来就默认PAUSE。此标记不因启动失败或重启而自动删除，后续所有启动保持安全暂停，只有新的显式控制请求才允许进入运行。旧指令缓存不跨进程恢复。上位机也不应在重连时自动发送RUN。

安装仅改变下次启动的默认值，不暂停或重启当前运行进程。首次加载新9010接口仍需安全停机后手动重启导航一次。

## 客户端命令

查询（不发送心跳/控制指令）：
```bash
python3 hmi_keyboard_client.py --host 127.0.0.1 --device-action status
```

先确认车辆停稳并暂停，再请求重启（会保持0.5秒心跳，等SCHEDULED后退出，不自动重试）：
```bash
python3 hmi_keyboard_client.py --host 127.0.0.1 --device-action restart --confirm-stopped
```

重启后用9011只读模式观察；上位机连接地址为Jetson当前IP。

宿主机服务nav-device-control.service开机启动、Unix socket 660/group docker，仅支持上述固定导航单位，不接受任意shell或服务名。它不属于导航服务的进程组，不会随导航停止。其持久任务状态也用于重启后查询。
