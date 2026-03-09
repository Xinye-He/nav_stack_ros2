builtin_interfaces/Time stamp

uint32 id           # CAN ID
bool   is_extended  # 是否扩展帧

uint8[8] data       # 原始8字节payload

# 方便人看的解码字段（和当前协议一一对应）
float32 dist_m      # 距离 (根据 dist_raw*0.2)
float32 steer_deg   # 转向角 (0=直行, 正=右转, 负=左转)
float32 speed_kmh   # 预速度 (根据speed_raw反算)

bool    pick
bool    unload
bool    dump
bool    pick_action
bool    remote
bool    estop
bool    drive
