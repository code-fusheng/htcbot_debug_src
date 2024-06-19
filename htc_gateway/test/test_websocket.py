#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncio
import websockets
import cv2

async def connect_to_websocket():
    # 124.223.72.28 192.168.1.121
    client_id = "test"
    vin = "HTCBOT_A0003"
    uri = f"ws://124.223.72.28:10240/robot/push/websocket?sid={client_id}&vin={vin}"  # 替换为你的 WebSocket 服务器的 URI
    
    target_fps = 5  # 目标帧率
    frame_skip = int(30 / target_fps)  # 跳过的帧数, 例如假设原始摄像头帧率为30fps

    async with websockets.connect(uri) as websocket:
        # 读取JPEG文件
        # with open("/Users/fusheng/WorkSpace/CompanyWork/work-fusheng/robot-pro/htcbot_debug_ws/src/htc_gateway/test/fusheng.jpg", "rb") as image_file:
        #     jpeg_bytes = image_file.read()

        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not cap.isOpened():
            print("Failed to open the webcam")
            return
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"Actual FPS: {actual_fps}")
        if actual_fps == 0:  # 如果无法获取实际帧率，则设置一个默认值
            actual_fps = 30
        frame_skip = int(actual_fps / target_fps)

        frame_count = 0

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame")
                break
            frame_count += 1
            # 只处理每第frame_skip帧
            if frame_count % frame_skip != 0:
                continue
            ret, jpeg_data = cv2.imencode('.jpg', frame)
            if not ret:
                print("Failed to encode frame")
                break
            jpeg_bytes = jpeg_data.tobytes()
        
            # print(jpeg_bytes)
            # 将JPEG数据推送到WebSocket服务器
            try:
                await websocket.send(jpeg_bytes)
                print("JPEG file sent successfully")
            except Exception as e:
                print(f"Failed to send JPEG file: {e}")
                return

        # 持续接收消息
        while True:
            response = await websocket.recv()
            print(f"Received: {response}")

# 运行WebSocket客户端
asyncio.get_event_loop().run_until_complete(connect_to_websocket())
