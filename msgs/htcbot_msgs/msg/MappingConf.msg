int8 mapping_state      # 建图状态
string save_dir         # 保存路径

float32 voxel_size      # 降采样
float32 step_size       # 步长
float32 add_shift       # 更新位移
bool is_flat            # 是否约束地面


int8 NONE_MAPPING = 0         # 待定(默认)
int8 START_MAPPING = 1  # 开始
int8 STOP_MAPPING = 2   # 暂停
int8 END_MAPPING = -1   # 结束 0|-1

bool YES = 1     # 是
bool NO = 0    # 否