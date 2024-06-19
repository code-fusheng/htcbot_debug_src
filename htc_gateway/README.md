sudo apt update
sudo apt install vnstat
sudo vnstat -u -i eth0
vnstat

vnstat -m
vnstat -h
vnstat -d

sudo apt install vnstati
vnstati -vs -i eth0 -o /path/to/output.png
vnstati -h -i eth0 -o /path/to/output.png
vnstati -d -i eth0 -o /path/to/output.png

vnstat -l -i eth0

# 车牌识别相机录制

ffmpeg -i rtsp://10.168.1.3:8557/h264 -c:v copy -c:a aac -b:a 128k -t 10 output.mp4
