"""Read-only host CAN polling and device-status enrichment."""
import asyncio
from contextlib import suppress
import json
import os
from pathlib import Path
import time


def enrich_can_status(item, bus, received, now, starting=False, error=''):
    result = dict(item, errors=list(item.get('errors', [])))
    age = None if received is None else max(0.0, now - received)
    result['bus'] = dict(bus or {}, interface='can0', status_age_ms=None if age is None else int(age * 1000))
    code, message = None, None
    if bus is None or age is None or age > 2.0:
        if starting:
            result['state'] = 'starting'
            return result
        code, message = 'CAN_BUS_STATUS_TIMEOUT', 'CAN 底层状态未更新：' + (error or '宿主机 CAN 管理服务未就绪或采集超时')
    elif bus.get('state') == 'MISSING':
        code, message = 'CAN_INTERFACE_MISSING', 'CAN 接口 can0 不存在'
    elif bus.get('state') == 'BUS-OFF':
        code, message = 'CAN_BUS_OFF', 'can0 已进入 BUS-OFF，无法正常收发（TX错误计数={}，RX错误计数={}）'.format(bus.get('tx_error_counter'), bus.get('rx_error_counter'))
    elif bus.get('interface_up') is False:
        code, message = 'CAN_INTERFACE_DOWN', 'CAN 接口 can0 未启用'
    elif bus.get('state') not in ('ERROR-ACTIVE', 'ERROR-WARNING', 'ERROR-PASSIVE'):
        code, message = 'CAN_BUS_STATUS_INVALID', '无法读取 CAN 底层状态：' + str(bus.get('error', bus.get('state')))
    if code:
        result['state'] = 'error'
        result['errors'].insert(0, dict(code=code, message=message, source='can0'))
    return result


async def poll_can_bus(node):
    default = Path(os.environ.get('ROS_WS', '/root/stack_can_ws')) / 'script/can_control/.runtime/control.sock'
    path = os.environ.get('CAN_CONTROL_SOCKET', str(default))
    while True:
        writer = None
        try:
            reader, writer = await asyncio.wait_for(asyncio.open_unix_connection(path), 1)
            request = dict(action='status', request_id='status-service-can')
            writer.write((json.dumps(request) + '\n').encode())
            await writer.drain()
            value = json.loads(await asyncio.wait_for(reader.readline(), 1.5))
            if not isinstance(value, dict) or not value.get('ok') or not isinstance(value.get('bus'), dict):
                raise ValueError('CAN 状态回执无效')
            node.can_bus_status = value['bus']
            node.can_bus_received = time.monotonic()
            node.can_bus_error = ''
        except (OSError, ValueError, asyncio.TimeoutError) as exc:
            node.can_bus_error = str(exc)
        finally:
            if writer:
                writer.close()
                with suppress(OSError):
                    await writer.wait_closed()
        await asyncio.sleep(0.5)
