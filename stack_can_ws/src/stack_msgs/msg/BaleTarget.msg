# 最近草捆目标信息（在雷达/车体坐标系下）

# 水平距离（沿地面投影的距离），单位：m
float32 distance_m

# 相对车体方向，单位：deg
# 约定：右正、左负（与你 StackCommand.angle_deg 一致）
# 例如：+20 表示草捆在车前方右侧约 20 度，-10 表示前左 10 度
float32 angle_deg

# 本帧是否有有效目标
bool valid
