# local planner

bool switch_to  # 开关
bool ON = 1     # 启动
bool OFF = 0    # 关闭

float32 sampling_tip_margin       # 中心点到水平采样起点距离
float32 sampling_out_margin       # 水平采样起点到平行采样起点的距离
float32 sampling_speed_factor     # 水平采样速度系数 
bool enable_heading_smoothing     # 启用航向平滑
bool enable_lane_change           # 启用车道变更

float32 max_local_plan_distance   # 局部最远规划距离
float32 horizon_distance    # 路径规划时前方距离
int8 rollouts_number         # 生成轨迹数
float32 rollouts_density     # 轨迹间距(m)

float32 horizontal_safety_distance     # 横向(水平)安全距离
float32 vertical_safety_distance       # 纵向(垂直)安全距离

float32 min_follow_distance            # 最小跟车距离
float32 lateral_skip_distance          # 轨迹生成跳跃步长

float32 path_density

float32 min_obstacle_evaluate_distance    # 最小障碍物评估范围



