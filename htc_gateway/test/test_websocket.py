#!/usr/bin/env python
# -*- coding: utf-8 -*-

# pip install websockets
# 

import asyncio
import websockets

async def connect_to_websocket():
    uri = "ws://localhost:10240/push/websocket"  # 替换为你的 WebSocket 服务器的 URI
    async with websockets.connect(uri) as websocket:
        # 发送消息
        await websocket.send("Hello, Server!")
        print("Sent: Hello, Server!")

        # 持续接收消息
        while True:
            response = await websocket.recv()
            print(f"Received: {response}")

asyncio.get_event_loop().run_until_complete(connect_to_websocket())
