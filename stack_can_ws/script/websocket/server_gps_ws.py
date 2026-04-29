#!/usr/bin/env python3
# server_gps_ws.py
# 功能：订阅 ROS 2 /gps/fix（BEST_EFFORT QoS），通过 WebSocket 广播 GPS 数据

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy
import asyncio
import websockets
import json
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('GPSWebSocketServer')

class GPSWebSocketServer(Node):
    def __init__(self):
        super().__init__('gps_websocket_server')
        
        # 配置 QoS：匹配 GPS 驱动（通常为 BEST_EFFORT）
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            qos
        )
        self.latest_data = None
        self.ws_clients = set()  # WebSocket 客户端集合（避免与 ROS 2 的 clients 冲突）
        logger.info("Subscribed to /fix with BEST_EFFORT QoS")

    def gps_callback(self, msg):
        # 过滤无效 GPS（status = -1 表示无定位）
        if msg.status.status == -1:
            self.latest_data = None
            return

        # 防止 altitude 为 NaN（某些驱动会发送 NaN）
        altitude = msg.altitude if not (msg.altitude != msg.altitude) else 0.0

        self.latest_data = {
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": altitude,
            "status": msg.status.status,
            "stamp_sec": msg.header.stamp.sec,
            "stamp_nsec": msg.header.stamp.nanosec
        }

async def register_client(websocket, ws_clients):
    """注册新客户端"""
    ws_clients.add(websocket)
    logger.info(f"Client connected. Total clients: {len(ws_clients)}")
    try:
        await websocket.wait_closed()
    finally:
        ws_clients.discard(websocket)
        logger.info(f"Client disconnected. Total clients: {len(ws_clients)}")

async def send_gps_data(node, ws_clients):
    """向所有连接的客户端广播 GPS 数据"""
    while rclpy.ok():
        if node.latest_data is not None and ws_clients:
            disconnected = set()
            for client in ws_clients:
                try:
                    await client.send(json.dumps(node.latest_data))
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(client)
            # 清理已断开的客户端
            ws_clients -= disconnected
        await asyncio.sleep(0.1)  # 10 Hz

async def ros_spin(node):
    """运行 ROS 2 节点（非阻塞）"""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.01)

async def main():
    rclpy.init()
    node = GPSWebSocketServer()
    port = 9001

    # 启动 WebSocket 服务器（监听所有接口）
    server = await websockets.serve(
        lambda ws, path: register_client(ws, node.ws_clients),
        "0.0.0.0",
        port,
        ping_interval=60,
        ping_timeout=30
    )

    logger.info(f"WebSocket server started on ws://0.0.0.0:{port}")
    logger.info(f"Remote clients connect to: ws://10.244.241.213:{port}")

    try:
        # 并发运行 ROS 和 WebSocket
        await asyncio.gather(
            ros_spin(node),
            send_gps_data(node, node.ws_clients),
            server.wait_closed()
        )
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
