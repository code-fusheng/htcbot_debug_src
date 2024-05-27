#!/bin/bash
###
 # @Author: code-fusheng
 # @Date: 2024-05-27 10:25:36
 # @LastEditors: code-fusheng 2561035977@qq.com
 # @LastEditTime: 2024-05-27 10:59:54
 # @Description: 
### 
# --exclude='.bin/app'
rsync --delete -avz --progress --exclude='.git' . robot@192.168.1.140:/home/robot/htcbot_debug_ws/src