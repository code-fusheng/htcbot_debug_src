#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# pip3 install pyserial
# pip3 install crcmod

import time
import device_model
import serial
import serial.tools.list_ports
import crcmod.predefined

class test_wit9073:

    def __init__(self):
        self.serial = None
        ports = list(serial.tools.list_ports.comports())

        for port, desc, hwid in sorted(ports):
            print(f'Port: {port}, Description: {desc}, Hardware ID: {hwid}')

        
    def updateData(device):
        print(
            "ID:{}  {}{}  AccX:{}  AccY:{}  AccZ:{}  AsX:{}  AsY:{}  AsZ:{}  AngX:{}  AngY:{}  AngZ:{}  Hx:{}  Hy:{}  Hy:{}"
            .format(device.get("CanID"), device.get("canmode_2"), device.get("canmode_1"), device.get("AccX"),
                    device.get("AccY"),
                    device.get("AccZ"), device.get("AsX"), device.get("AsY"), device.get("AsZ"), device.get("AngX"),
                    device.get("AngY"), device.get("AngZ"), device.get("HX"), device.get("HY"), device.get("HZ")))

    def open_com(self, port, baud):
        try:
            self.serial = serial.Serial(port=port, baudrate=baud, timeout=0.2)
        except Exception as e:
            print(f"无法打开串口: {e}")
            exit()

        while True:
            response = self.serial.read_until(expected=b'\x01')  # 假设响应以地址0x01开始，这里可以根据实际情况修改
            if response:
                print(f"收到的原始响应: {response.hex()}")                    
            time.sleep(0.1)  # 小的延迟，避免高CPU使用率

if __name__ == '__main__':
    app = test_wit9073()
    app.open_com("/dev/cu.usbserial-14330", 2000000)
    