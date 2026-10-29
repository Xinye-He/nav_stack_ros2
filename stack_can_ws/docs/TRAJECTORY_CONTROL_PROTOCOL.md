# 上位机路径重载与循迹启动

使用已有9010长连接，持续发送应用心跳。文件上传、路径重载、循迹启动是三个独立步骤。

## 1. 上传

使用9002/points上传CSV，收到saved回执仅表示文件保存。此功能不重载电子围栏，也不更新fence.csv的运行节点。

## 2. 重载（不启动）

```json
{"type":"trajectory","action":"reload","request_id":"reload-001"}
```

执行节点先暂停并替换为全零手动指令，再要求循迹节点读取当前配置的path_csv。
严格检查数据行、有限经纬度/航向及索引重复，构建完成后才替换运行路径，不静默跳过坏行。
失败时保留旧路径几何但停在暂停/原急停，启动许可失效。成功后从第一个路点重新计进度。
不会解除急停、中止或切换RUN。已有急停时仍保持急停，不伪装成已解除。

```json
{
  "type":"trajectory_result",
  "request_id":"reload-001",
  "action":"reload",
  "ok":true,
  "code":"OK",
  "message":"路径已重载，未启动行驶",
  "drive_state":0,
  "path_version":"64位SHA256",
  "waypoint_count":20,
  "path_file":"/root/stack_can_ws/data/points.csv"
}
```

path_version是此次实际读取CSV字节的SHA256，path_file是当前配置的文件路径，上位机不能通过请求任意指定文件。
新的原点建立后必须等待新的定位与航向更新。保留reload成功回执中的path_version用于后续start。

## 3. 显式启动

```json
{"type":"trajectory","action":"start","request_id":"start-001","path_version":"重载回执中的64位SHA256"}
```

必须本次执行节点运行期间通过新接口成功重载。需满足：
- 无急停、中止、实体遥控接管。
- 围栏、9010链路、必要外设/控制节点就绪且状态新鲜。
- 路径版本匹配，磁盘文件未在加载后变更。
- 循迹节点有新鲜、有效的新坐标系定位与航向。
- 执行节点已收到新鲜、有效的循迹停止指令。

循迹节点先回复准备完成而不RUN，执行节点再次检查保护条件，通过后才切换为自动控制并发布RUN。

```json
{
  "type":"trajectory_result",
  "request_id":"start-001",
  "action":"start",
  "ok":true,
  "code":"OK",
  "message":"循迹启动已确认",
  "drive_state":1,
  "path_version":"64位SHA256",
  "waypoint_count":20
}
```

这是执行节点接受并应用启动的确认，不证明实际运动、路径安全或达到目标速度。
启动后通过9011的vehicle_state及实际车辆反馈观察；历史runtime_fault可能仍保留。

## 4. 回执及错误

同一9010连接返回trajectory_result。字段：type、request_id、action、ok、code、message，执行节点结果还含drive_state。
路径点数、版本、文件路径按结果可选提供。新操作使用新request_id，不提供持久去重缓存。

| code | 含义 |
|---|---|
| OK | 重载成功或启动确认 |
| INVALID_REQUEST | action或request_id非法 |
| TRAJECTORY_BUSY | 另一条路径操作正在执行 |
| PATH_RELOAD_FAILED | 读取/解析/构建路径失败 |
| TRAJECTORY_START_REJECTED | 未重载、版本不匹配、定位航向未就绪、急停或保护条件不满足等 |
| TRAJECTORY_TIMEOUT | 等待执行/循迹节点超时，结合message和实际状态确认 |
| PATH_CONTROL_FAILED | 内部路径操作失败的兜底错误 |

读取回执前不要自动启动。执行节点等内部回复最多8秒，桥接端等执行结果最多10秒，期间心跳不能停止。
断线/超时不应自动重发start，也不能当作启动成功。

## 5. 键盘测试

```bash
python3 hmi_keyboard_client.py --host 127.0.0.1 --full-control
```

- n：重载路径，等待成功并记住返回的版本。
- y：显式启动刚重载的路径，没有成功重载回执则不发送。
- z：原PAUSE。
- x：原急停。

默认work-only模式屏蔽n和y。旧teleop_key.py的n及/restart_path兼容入口仍保持原行为，会自动进入RUN；新上位机不要使用旧入口。
普通u/g等旧控制接口也保持兼容，不具备新路径启动接口的版本及结果确认。

修改main包后需编译，并重启导航服务让通信、执行和循迹进程加载新代码；实现及测试过程不自动重启或发送启动。
