#!/usr/bin/env python3
# server_heading_ws.py
# 功能：订阅 ROS 2 /gps/heading_deg（BEST_EFFORT QoS），通过 WebSocket 广播车辆航向角数据

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy
import asyncio
import websockets
import json
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('HeadingWebSocketServer')

class HeadingWebSocketServer(Node):
    def __init__(self):
        super().__init__('heading_websocket_server')

        # 配置 QoS：匹配 GPS 驱动（通常为 BEST_EFFORT）
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            Float32,
            '/heading_deg',
            self.heading_callback,
            qos
        )
        self.latest_heading = None
        self.ws_clients = set()  # WebSocket 客户端集合
        logger.info("Subscribed to /heading_deg with BEST_EFFORT QoS")

    def heading_callback(self, msg):
        # 接收航向角（单位：度）
        self.latest_heading = msg.data

async def register_client(websocket, ws_clients):
    """注册新客户端"""
    ws_clients.add(websocket)
    logger.info(f"Heading client connected. Total clients: {len(ws_clients)}")
    try:
        await websocket.wait_closed()
    finally:
        ws_clients.discard(websocket)
        logger.info(f"Heading client disconnected. Total clients: {len(ws_clients)}")

async def send_heading_data(node, ws_clients):
    """向所有连接的客户端广播航向角数据"""
    while rclpy.ok():
        if node.latest_heading is not None and ws_clients:
            disconnected = set()
            message = json.dumps({
                "heading_deg": node.latest_heading
            })
            for client in ws_clients:
                try:
                    await client.send(message)
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(client)
            ws_clients -= disconnected
        await asyncio.sleep(0.1)  # 10 Hz

async def ros_spin(node):
    """运行 ROS 2 节点（非阻塞）"""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.01)

async def main():
    rclpy.init()
    node = HeadingWebSocketServer()
    port = 9003# 建议使用与 GPS 不同的端口避免冲突

    # 启动 WebSocket 服务器（监听所有接口）
    server = await websockets.serve(
        lambda ws, path: register_client(ws, node.ws_clients),
        "0.0.0.0",
        port,
        ping_interval=60,
        ping_timeout=30
    )

    logger.info(f"WebSocket heading server started on ws://0.0.0.0:{port}")
    logger.info(f"Remote clients connect to: ws://10.244.241.213:{port}")

    try:
        # 并发运行 ROS 和 WebSocket
        await asyncio.gather(
            ros_spin(node),
            send_heading_data(node, node.ws_clients),
            server.wait_closed()
        )
    except KeyboardInterrupt:
        logger.info("Shutting down heading server...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
