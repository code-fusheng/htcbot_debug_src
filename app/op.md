<!-- error -->

crontab -e
@reboot /home/robot/htcbot_online_ws/src/.bin/app/htcbot_startup.sh

<!-- 开机自启动 -->

sudo vim /etc/systemd/system/htcbot.service
sudo systemctl daemon-reload
sudo systemctl enable htcbot.service
sudo systemctl start htcbot.service
sudo systemctl status htcbot.service
sudo journalctl -xe
Restart=no
Restart=on-failure

<!--  -->

sudo apt-get install ros-$ROS_DISTRO-robot-upstart
rosrun robot_upstart install auto_start/launch/auto_start.launch
sudo systemctl daemon-reload && sudo systemctl start auto
sudo apt-get install setpriv
