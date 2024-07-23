#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import paho.mqtt.client as mqtt
import json
import base64
from datetime import datetime, timedelta

class TestLocalGateway:

    def __init__(self):
        self.mqtt_client = None
        # pro 
        self.camera_pro_heartbeat = "/patrol_robot/up/device/camera/pro_heartbeat"
        self.camera_pro_lpr_result = "/patrol_robot/up/device/camera/pro_lpr_result"
        # zhenshi
        self.camera_heartbeat = "/patrol_robot/up/device/camera/heartbeat"
        self.camera_ivs_result = "/patrol_robot/up/device/camera/ivs_result"
        self.client_id = "test_local"
        self.broker = "118.190.156.22"
        self.port   = 1883
        self.username = "htcbot"
        self.password = "htcbot123456"
        self.init_local()
        pass

    def init_local(self):
        self.mqtt_client = mqtt.Client(client_id=self.client_id)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.username_pw_set(self.username, self.password)

    def on_connect(self, client, userdata, flags, rc):
        print(f"[test_local_gateway] ===> mqtt Connected with result code: {str(rc)}")
        # 0: 连接成功
        # 1: 连接被拒绝，不支持的协议版本
        # 2: 连接被拒绝，不合格的client identifier
        # 3: 连接被拒绝，服务器不可用
        # 4: 连接被拒绝，错误的用户名或密码
        # 5: 连接被拒绝，未授权
        if rc == 0:
            self.mqtt_client.subscribe(self.camera_pro_heartbeat)
            self.mqtt_client.subscribe(self.camera_pro_lpr_result)
            self.mqtt_client.subscribe(self.camera_heartbeat)
            self.mqtt_client.subscribe(self.camera_ivs_result)
        else:
            print(f"Failed to connect, return code {rc}")

    def on_message(self, client, userdata, msg):
        print(f"[test_local_gateway] ===> Received message on topic {msg.topic}: {msg.payload.decode()}")
        # 根据话题选择文件
        topic_to_file_map = {
            "/patrol_robot/up/device/camera/pro_heartbeat": "camera_pro_heartbeat.txt",
            "/patrol_robot/up/device/camera/pro_lpr_result": "camera_pro_lpr_result.txt",
            "/patrol_robot/up/device/camera/ivs_result": "camera_ivs_result.txt",
            "/patrol_robot/up/device/camera/heartbeat": "camera_heartbeat.txt",
        }
        message = json.loads(msg.payload.decode())
        # 获取对应文件名
        file_name = topic_to_file_map.get(msg.topic, "default.txt")  # 如果话题不在映射中，使用默认文件
        if msg.topic == "/patrol_robot/up/device/camera/ivs_result":
            if 'payload' in message and 'AlarmInfoPlate' in message['payload']:
                alarm_info = message['payload']['AlarmInfoPlate']
                if 'result' in alarm_info and 'PlateResult' in alarm_info['result']:
                    plate_result = alarm_info['result']['PlateResult']
                    encoded_license = plate_result.get('license', '')
                    # 解码 Base64 车牌字段
                    decoded_bytes = base64.b64decode(encoded_license)
                    decoded_license = decoded_bytes.decode('utf-8')
                    message['payload']['license'] = decoded_license
                    # 获取时间戳字段
                    timeval = plate_result.get('timeStamp', {}).get('Timeval', {})
                    sec = timeval.get('sec', 0)
                    usec = timeval.get('usec', 0)
                    # 转换为时间戳
                    timestamp = datetime.fromtimestamp(sec) + timedelta(microseconds=usec)
                    formatted_timestamp = timestamp.strftime('%Y-%m-%d %H:%M:%S')
                    message['payload']['timestamp'] = formatted_timestamp
                    message['payload']['AlarmInfoPlate'] = {}
        # 将消息写入对应的文件
        with open(file_name, "a") as file:
            # file.write(msg.payload.decode() + "\n")
            file.write(json.dumps(message, ensure_ascii=False) + "\n")
            
    def connect(self):
        try:
            self.mqtt_client.connect(self.broker, self.port, 60)
            self.mqtt_client.loop_start()
            print("[test_local_gateway] ===> Connected to MQTT Broker")
        except Exception as e:
            print(f"Error connecting to MQTT Broker: {e}")

    def disconnect(self):
        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            print("[test_local_gateway] ===> Disconnected from MQTT Broker")
        except Exception as e:
            print(f"Error disconnecting from MQTT Broker: {e}")

if __name__ == '__main__':
    app = TestLocalGateway()
    app.connect()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        app.disconnect()