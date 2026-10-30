"""Isolated ROS + WebSocket integration. enable_can=false; no CAN devices."""
import asyncio,json,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/main'))
import rclpy
from websockets.legacy.client import connect
from websockets.legacy.server import serve
from main.stack_can_executor import StackCanExecutor
from main.websocket_teleop_key import WebSocketTeleopBridge

async def run():
    rclpy.init(args=['--ros-args','-p','enable_can:=false'])
    executor=StackCanExecutor();bridge=WebSocketTeleopBridge()
    assert not executor.enable_can and executor.can_bus is None
    async def spin():
        while rclpy.ok():
            rclpy.spin_once(executor,timeout_sec=0)
            rclpy.spin_once(bridge,timeout_sec=0)
            executor._update_safety_state();executor._update_control_mode()
            await asyncio.sleep(.005)
    task=asyncio.create_task(spin())
    async def until(predicate):
        for _ in range(250):
            if predicate():return
            await asyncio.sleep(.01)
        raise AssertionError('condition timed out')
    try:
        async with serve(bridge.handle_client,'127.0.0.1',19010):
            # Query-only connection: no control session, no network fault.
            async def query(ws,data):await ws.send(json.dumps(dict(type='can_result',ok=True,action='status')))
            original_submit=bridge.can_control.submit;bridge.can_control.submit=query
            async with connect('ws://127.0.0.1:19010') as ws:
                await ws.send(json.dumps(dict(type='can',action='status',request_id='query')))
                assert json.loads(await ws.recv())['ok']
            await asyncio.sleep(.4)
            assert not bridge.network_fault_latched and bridge.network_event_pending is None
            bridge.can_control.submit=original_submit
            # Initial bridge broadcast has settled; select trajectory in executor.
            executor.drive_state=1;executor.teleop_active=False;executor.bale_active=False
            executor.cmd_traj=executor._make_stop_command();executor.cmd_traj.pre_speed_kmh=4.
            async with connect('ws://127.0.0.1:19010') as ws:
                await ws.send(json.dumps(dict(type='heartbeat')))
                await until(lambda:bridge.ws_connected)
            await until(lambda:bridge.network_warning_latched)
            assert executor.drive_state==1 and not executor.estop_latched and executor.cmd_traj.pre_speed_kmh==4.
            print('PASS: trajectory disconnect warns and keeps RUN',flush=True)
            # Switch actual executor selection to 9010 teleop, then disconnect.
            executor.teleop_active=True;executor.cmd_teleop=executor._make_stop_command();executor.teleop_source='websocket_9010'
            async with connect('ws://127.0.0.1:19010') as ws:
                await ws.send(json.dumps(dict(type='heartbeat')))
                await until(lambda:bridge.ws_connected)
            await until(lambda:executor.estop_latched)
            assert executor.estop_reason=='network_disconnected'
            print('PASS: teleop disconnect latches network ESTOP',flush=True)
            async with connect('ws://127.0.0.1:19010') as ws:
                await ws.send(json.dumps(dict(type='heartbeat')))
                await until(lambda:bridge.ws_connected)
                assert executor.estop_latched
                executor.paused_reset_guards={name:(True,time.monotonic()) for name in ('围栏','9010链路','外设及控制节点')}
                await ws.send(json.dumps(dict(type='reset_estop_paused',request_id='manual-reset')))
                result=json.loads(await asyncio.wait_for(ws.recv(),3))
                assert result['ok'] and result['drive_state']==0 and result['cleared_reason']=='network_disconnected'
                assert not executor.estop_latched and executor.drive_state==0
                print('PASS: reconnect alone keeps ESTOP; manual reset ends PAUSED',flush=True)
    finally:
        task.cancel();await asyncio.gather(task,return_exceptions=True)
        bridge.disable_raw();bridge.destroy_node();executor.destroy_node();rclpy.shutdown()
asyncio.run(run())
