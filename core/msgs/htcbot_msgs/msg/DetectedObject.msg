# 感知对象

std_msgs/Header                 header

uint32                          id
string                          label
float32                         score
std_msgs/ColorRGBA              color
bool                            valid

# 3D 对象
string                          space_frame

sensor_msgs/PointCloud2         pointcloud

geometry_msgs/Pose              pose            # 对象的位姿信息
geometry_msgs/Vector3           dimensions      # 对象的三维尺寸
geometry_msgs/PolygonStamped    convex_hull     # 表示对象的凸包，即包围对象的最小凸多边形