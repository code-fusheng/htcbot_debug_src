#!/bin/bash

cd /home/${USER}/htcbot_debug_ws
source devel/setup.bash
sleep 10
# roslaunch turn_htcbot turn_on.launch start_key:=${START_KEY} & exec bash
roslaunch demo_tools py_demo.launch & exec bash