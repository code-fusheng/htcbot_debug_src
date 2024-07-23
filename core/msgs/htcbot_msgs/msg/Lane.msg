Header header

int32 lane_id               # 车道的唯一标识符
Waypoint[] waypoints        # 车道上的路径点

int32 lane_index           # 车道的索引
float64 cost                # 车道相关的成本或代价
float64 center_cost         # 车道中心线的成本或代价
float64 transition_cost     # 车道转换的成本或代价
float64 lateral_cost        # 横向成本或代价
float64 long_cost           # 纵向成本或代价

float64 closest_object_distance     # 最接近车道的障碍物的距离
float64 closest_object_velocity     # 最接近车道的障碍物的速度
bool is_blocked             # 车道是否被阻塞
int32 best_rollout_index    # 最佳规划路径的索引