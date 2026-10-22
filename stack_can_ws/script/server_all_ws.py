#!/usr/bin/env python3
# server_all_ws.py
# 功能：
# 1. 订阅 ROS 2 /fix，通过 ws://IP:9001 广播 GPS 数据
# 2. 接收 ws://IP:9002/fence 和 ws://IP:9002/points 上传的 CSV，保存为 fence.csv / points.csv
# 3. 订阅 ROS 2 /heading_deg，通过 ws://IP:9003 广播航向角数据
# 4. ws://IP:9010 复用 websocket_teleop_key 的远控和断联保护

import asyncio
import importlib.util
from contextlib import AsyncExitStack
import csv
import io
import json
import logging
import math
import pathlib
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

from websockets.legacy.server import serve
from websockets.exceptions import ConnectionClosed


# =========================
# 基本配置
# =========================

HOST = "0.0.0.0"

GPS_WS_PORT = 9001
CSV_WS_PORT = 9002
HEADING_WS_PORT = 9003
TELEOP_WS_PORT = 9010

GPS_TOPIC = "/fix"
HEADING_TOPIC = "/heading_deg"

SAVE_DIR = pathlib.Path("/root/stack_can_ws/data")

GPS_SEND_PERIOD = 0.1       # 10 Hz
HEADING_SEND_PERIOD = 0.1   # 10 Hz
ROS_SPIN_PERIOD = 0.01

PING_INTERVAL = 60
PING_TIMEOUT = 30

logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] [%(name)s] %(message)s"
)
logger = logging.getLogger("ServerAllWS")


# =========================
# ROS2 节点
# =========================

class VehicleDataWebSocketNode(Node):
    def __init__(self):
        super().__init__("vehicle_data_websocket_server")

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            GPS_TOPIC,
            self.gps_callback,
            qos
        )

        self.heading_sub = self.create_subscription(
            Float32,
            HEADING_TOPIC,
            self.heading_callback,
            qos
        )

        self.latest_gps = None
        self.latest_heading = None

        self.gps_clients = set()
        self.heading_clients = set()

        logger.info(f"Subscribed to {GPS_TOPIC} with BEST_EFFORT QoS")
        logger.info(f"Subscribed to {HEADING_TOPIC} with BEST_EFFORT QoS")

    def gps_callback(self, msg: NavSatFix):
        # status = -1 表示无定位
        if msg.status.status == -1:
            self.latest_gps = None
            return

        # 防止 altitude 为 NaN
        altitude = msg.altitude
        if isinstance(altitude, float) and math.isnan(altitude):
            altitude = 0.0

        self.latest_gps = {
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": altitude,
            "status": msg.status.status,
            "stamp_sec": msg.header.stamp.sec,
            "stamp_nsec": msg.header.stamp.nanosec,
        }

    def heading_callback(self, msg: Float32):
        self.latest_heading = float(msg.data)


# =========================
# WebSocket 兼容处理
# =========================

def get_ws_path(websocket, path: Optional[str]) -> str:
    """
    兼容不同 websockets 版本：
    - 老版本 handler(websocket, path)
    - 新版本 handler(websocket)
    """
    if path is not None:
        return path

    # websockets 新版本可能放在 websocket.request.path
    request = getattr(websocket, "request", None)
    if request is not None:
        req_path = getattr(request, "path", None)
        if req_path:
            return req_path

    # 老版本有时 websocket.path 可用
    ws_path = getattr(websocket, "path", None)
    if ws_path:
        return ws_path

    return "/"


# =========================
# GPS WebSocket
# =========================

async def gps_ws_handler(websocket, path=None):
    node: VehicleDataWebSocketNode = gps_ws_handler.node

    node.gps_clients.add(websocket)
    logger.info(f"GPS client connected. Total GPS clients: {len(node.gps_clients)}")

    try:
        await websocket.wait_closed()
    finally:
        node.gps_clients.discard(websocket)
        logger.info(f"GPS client disconnected. Total GPS clients: {len(node.gps_clients)}")


async def broadcast_gps(node: VehicleDataWebSocketNode):
    while rclpy.ok():
        if node.latest_gps is not None and node.gps_clients:
            message = json.dumps(node.latest_gps)
            disconnected = set()

            for client in list(node.gps_clients):
                try:
                    await client.send(message)
                except ConnectionClosed:
                    disconnected.add(client)
                except Exception as e:
                    logger.warning(f"Send GPS failed: {e}")
                    disconnected.add(client)

            node.gps_clients -= disconnected

        await asyncio.sleep(GPS_SEND_PERIOD)


# =========================
# Heading WebSocket
# =========================

async def heading_ws_handler(websocket, path=None):
    node: VehicleDataWebSocketNode = heading_ws_handler.node

    node.heading_clients.add(websocket)
    logger.info(
        f"Heading client connected. Total heading clients: {len(node.heading_clients)}"
    )

    try:
        await websocket.wait_closed()
    finally:
        node.heading_clients.discard(websocket)
        logger.info(
            f"Heading client disconnected. Total heading clients: {len(node.heading_clients)}"
        )


async def broadcast_heading(node: VehicleDataWebSocketNode):
    while rclpy.ok():
        if node.latest_heading is not None and node.heading_clients:
            message = json.dumps({
                "heading_deg": node.latest_heading
            })
            disconnected = set()

            for client in list(node.heading_clients):
                try:
                    await client.send(message)
                except ConnectionClosed:
                    disconnected.add(client)
                except Exception as e:
                    logger.warning(f"Send heading failed: {e}")
                    disconnected.add(client)

            node.heading_clients -= disconnected

        await asyncio.sleep(HEADING_SEND_PERIOD)


# =========================
# CSV WebSocket
# =========================

async def csv_ws_handler(websocket, path=None):
    ws_path = get_ws_path(websocket, path)

    if ws_path == "/fence":
        expected_file = "fence.csv"
        min_columns = 3
        expected_header = ["idx", "lat", "lon"]

    elif ws_path == "/points":
        expected_file = "points.csv"
        min_columns = 6
        expected_header = None

    else:
        await websocket.send("error: unsupported path. Use /fence or /points")
        logger.warning(f"CSV client used unsupported path: {ws_path}")
        return

    logger.info(f"CSV client connected on path {ws_path}, target file: {expected_file}")

    async for message in websocket:
        if not isinstance(message, str):
            await websocket.send("error: expected text CSV payload")
            continue

        try:
            reader = csv.reader(io.StringIO(message))
            rows = [row for row in reader]
        except Exception as e:
            await websocket.send(f"error: csv parse failed: {e}")
            continue

        if not rows:
            await websocket.send("error: empty CSV")
            continue

        # fence.csv 强制校验 header
        if expected_header is not None:
            if rows[0] != expected_header:
                await websocket.send(
                    f"error: invalid header. Expected: {expected_header}, got: {rows[0]}"
                )
                continue
            data_rows = rows[1:]
        else:
            data_rows = rows

        if not data_rows:
            await websocket.send("error: CSV has header but no data rows")
            continue

        if not all(len(row) >= min_columns for row in data_rows):
            await websocket.send(
                f"error: some rows have fewer than {min_columns} columns"
            )
            continue

        SAVE_DIR.mkdir(parents=True, exist_ok=True)
        save_path = SAVE_DIR / expected_file
        tmp_path = SAVE_DIR / f".{expected_file}.tmp"

        try:
            # 先写临时文件，再原子替换，避免写到一半被其他节点读取
            with tmp_path.open("w", encoding="utf-8", newline="") as f:
                f.write(message)

            tmp_path.replace(save_path)

        except Exception as e:
            await websocket.send(f"error: save {expected_file} failed: {e}")
            logger.error(f"Save {expected_file} failed: {e}")
            continue

        await websocket.send(
            f"saved:{expected_file}, rows={len(rows)}, valid=True"
        )
        logger.info(f"Saved {save_path}, rows={len(rows)}")


# =========================
# ROS spin
# =========================

def load_teleop_bridge():
    # Load the workspace source, avoiding a stale installed copy.
    source = pathlib.Path(__file__).resolve().parents[1] / "src/main/main/websocket_teleop_key.py"
    spec = importlib.util.spec_from_file_location("server_all_teleop", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class RemoteOnlyBridge(module.WebSocketTeleopBridge):
        def enable_raw(self):
            # This service runs in the background; do not capture terminal input.
            return False

    return RemoteOnlyBridge


async def ros_spin(*nodes):
    while rclpy.ok():
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.0)
        await asyncio.sleep(ROS_SPIN_PERIOD)


async def main():
    bridge_class = load_teleop_bridge()
    rclpy.init()
    node = None
    bridge = None
    tasks = []
    try:
        # Bind every port before constructing the control node, which publishes
        # initial control state. A port conflict must fail before publication.
        async def teleop_handler(websocket, path=None):
            await bridge.handle_client(websocket, path)

        async with AsyncExitStack() as servers:
            for handler, port in (
                (gps_ws_handler, GPS_WS_PORT),
                (csv_ws_handler, CSV_WS_PORT),
                (heading_ws_handler, HEADING_WS_PORT),
                (teleop_handler, TELEOP_WS_PORT),
            ):
                await servers.enter_async_context(serve(
                    handler, HOST, port,
                    ping_interval=20 if port in (CSV_WS_PORT, TELEOP_WS_PORT) else PING_INTERVAL,
                    ping_timeout=10 if port in (CSV_WS_PORT, TELEOP_WS_PORT) else PING_TIMEOUT,
                ))
            node = VehicleDataWebSocketNode()
            bridge = bridge_class()
            gps_ws_handler.node = node
            heading_ws_handler.node = node
            logger.info("WebSocket services listening on %s: GPS=%s CSV=%s heading=%s teleop=%s",
                        HOST, GPS_WS_PORT, CSV_WS_PORT, HEADING_WS_PORT, TELEOP_WS_PORT)
            logger.info("Teleop uses websocket_teleop_key JSON commands; send messages within %.2fs",
                        bridge.link_timeout_sec)
            tasks = [asyncio.create_task(coro) for coro in (
                ros_spin(node, bridge), broadcast_gps(node), broadcast_heading(node)
            )]
            try:
                await asyncio.gather(*tasks)
            finally:
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
    finally:
        if bridge is not None:
            bridge.disable_raw()
            bridge.destroy_node()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Server stopped by user")
