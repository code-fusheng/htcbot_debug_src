# sudo apt-get install wmctrl

RVIZ_WINDOW_ID=$(wmctrl -l | grep "RViz" | awk '{print $1}')
# 调整窗口大小和位置（1920x1080），如果需要设置特定位置（例如左上角），可以使用 `-e` 选项
wmctrl -ir $RVIZ_WINDOW_ID -e 0,0,0,1920,1080

