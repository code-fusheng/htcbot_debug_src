# 维特智能驱动手册

https://pan.baidu.com/s/1PVfmRWPc4259qZ87InQbGQ?pwd=t6ab 密码空格

### 设备编号

- HWT-9073
  Bus 003 Device 011: ID 1a86:7523 QinHeng Electronics HL-340 USB-Serial adapter

```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="imu_wit9073"
```

mac
Port: /dev/cu.usbserial-14330, Description: USB Serial, Hardware ID: USB VID:PID=1A86:7523 LOCATION=20-3.3
