#!/bin/bash
###
 # @Author: code-fusheng
 # @Date: 2024-04-21 18:41:37
 # 192.168.3.62
 # 10.168.1.194
 # 172.20.10.4 iphone
 # 192.168.1.128
 # 192.168.2.102
### 

rsync -avz --progress --exclude='.git' --exclude='temp' . fusheng@192.168.2.32:/Users/fusheng/WorkSpace/CompanyWork/work-fusheng/robot-pro/htcbot_debug_ws/src