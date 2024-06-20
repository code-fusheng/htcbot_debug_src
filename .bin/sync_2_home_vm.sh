#!/bin/bash
# # waka_6366cf14-6d76-4721-8712-01423fd92a4b
# 192.168.2.14

# rsync -avz --progress --exclude='.git' . htc@192.168.1.111:/home/htc/htcbot_ws/src

rsync --delete -avz --progress --exclude='.git' . code@192.168.2.14:/home/code/htcbot_debug_ws/src
