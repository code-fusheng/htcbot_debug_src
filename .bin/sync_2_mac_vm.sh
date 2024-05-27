#!/bin/bash
###
 # @Author: code-fusheng
 # @Date: 2024-04-21 18:41:37
 # # waka_6366cf14-6d76-4721-8712-01423fd92a4b
 # 192.168.2.106
 # 172.20.10.5 
### 

rsync --delete -avz --progress --exclude='.git' . code@192.168.2.109:/home/code/htcbot_debug_ws/src