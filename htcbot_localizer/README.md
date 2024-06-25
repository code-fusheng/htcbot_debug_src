<!--
/*
 * @Author:lizhenglei01
 * @Date: 2024-06-24 22:10:49
 * @LastEditors: lizhenglei01   2207076701@qq.com
 * @LastEditTime: 2024-06-25 16:10:49 16:01:04
 * @Description: 
 */
-->






### imu_ndt 定位测试

```

  加载地图

   auto_start.py下  设置地图路径   self.base_dir = "/home/lzl/datas"



  数据集可视化


    roslaunch   htcbot_localizer imu_and_ndt_localizer.launch 
    
    修改以下参数

    <arg name="is_auto" default="true" />
    <arg name="is_debug" default="true" />






   

```

