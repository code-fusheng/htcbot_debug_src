#!/bin/bash
###
 # @Author: code-fusheng
 # @Date: 2024-04-21 18:41:37
 # 192.168.2.106
 # 172.20.10.5 
 # 10.168.1.107
### 

rsync --delete -avz --progress --exclude='.git' . code@172.20.10.5:/home/code/htcbot_debug_ws/src