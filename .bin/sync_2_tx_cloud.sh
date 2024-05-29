#!/bin/bash
###
 # @Author: code-fusheng
 # @Date: 2024-05-27 10:25:36
 # @LastEditors: code-fusheng 2561035977@qq.com
 # @LastEditTime: 2024-05-28 01:03:30
 # @Description: 
### 
# --exclude='.bin/app'
rsync --delete -avz --progress --exclude='.git' . ubuntu@124.223.72.28:/home/ubuntu/htcbot_cloud_ws/src