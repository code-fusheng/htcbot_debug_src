#!/bin/bash
###
 # @Author: code-fusheng
 # @Date: 2024-04-21 18:41:37
 # 192.168.2.106 192.168.2.110
 # 172.20.10.5 
 # 192.168.1.141
### 

rsync --delete -avz --progress --exclude='.git' . code@192.168.1.109:/home/code/htcbot_debug_ws/src