int32 gid 
int32 lid

float32 speed_limit     # 车道或路口的速度限制 m/s
float32 yaw             # 车道或路口的航向角，表示方向
int16 is_lane           # 表示是直线还是转弯路口的标志 lane = 1 , cross = 0
int16 lane_id           # 车道的ID
int16 lane_index        # 车道索引
uint32 left_lane_id
uint32 right_lane_id
uint32 stop_line_id

int16 cross_id          # 转弯路口的ID
float32 cost

geometry_msgs/PoseStamped pose      # 车道或路口的姿态信息