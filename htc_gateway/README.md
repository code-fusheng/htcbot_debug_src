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
