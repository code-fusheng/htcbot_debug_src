## Self Defined NMEA Driver

## 华芯 GPS:

- 默认波特率 460800
- 标准组合导航输出 GHFPD

## 中海达 GPS

- 默认波特率 230400
- 标准组合导航输出 GINS

## D300

频率设置示例:gpgga 1
1 表示 1 秒一次，设置 10HZ 就是:gpgga 0.1
saveconfig
set ntrip 103.143.19.54 8002 RTCM33GRCEJpro 6a1977 62637
set RTCM ServiceProvider ntrip
