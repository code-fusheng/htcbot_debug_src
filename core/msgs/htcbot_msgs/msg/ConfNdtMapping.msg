# Conf Mapping

int16 switch_to_exp    # 状态(拓展)
int16 NONE = 0         # 待定(默认)
int16 START = 1        # 开始
int16 FINISH = -1      # 结束
int16 STOP = 2         # 暂停

int16 method_type       #

# NDT
float32 voxel_leaf_size     # 降采样
float32 step_size           # 步长
int16 max_iter              # 最大迭代次数
float32 ndt_res             # 地图分辨率
float32 trans_eps           # 变换差值

bool is_limit_flat          # 是否限制地面

bool is_filter_add_map    # 是否过滤后添加
float32 filter_res        

bool is_sync_update_map      # 是否同步更新地图

bool is_remapping       # 是否地图重建

bool YES = 1     # 是
bool NO = 0    # 否

float32 min_add_shift       # 更新位移

bool use_odom           # 是否使用 odom
bool use_imu            # 是否使用 imu
bool use_gnss           # 是否使用 gnss
bool use_vo             # 是否使用 vo
bool use_vehicle

float32 min_scan_range
float32 max_scan_range
float32 min_scan_height
float32 max_scan_height







