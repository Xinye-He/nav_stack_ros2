"""Private ROS/WebSocket/Unix-socket integration; systemctl is a mock."""
import asyncio,importlib.util,json,os,sys,tempfile
from pathlib import Path
from unittest.mock import AsyncMock,patch
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/main'))
import rclpy
from websockets.legacy.client import connect
from websockets.legacy.server import serve
from main.stack_can_executor import StackCanExecutor
from main.websocket_teleop_key import WebSocketTeleopBridge
spec=importlib.util.spec_from_file_location('restart_host',ROOT/'script/device_control/service.py');host=importlib.util.module_from_spec(spec);spec.loader.exec_module(host)
async def run():
    with tempfile.TemporaryDirectory() as temp,patch.dict(os.environ,{'NAV_START_PAUSED':'1','DEVICE_CONTROL_SOCKET':str(Path(temp)/'control.sock')}):
        restart=AsyncMock();controller=host.Controller(temp,restart,delay=.05)
        async def handler(reader,writer):
            value=json.loads(await reader.readline());result=await controller.execute(value)
            writer.write((json.dumps(result)+'\n').encode());await writer.drain();writer.close();await writer.wait_closed()
        unix=await asyncio.start_unix_server(handler,path=os.environ['DEVICE_CONTROL_SOCKET'])
        rclpy.init(args=['--ros-args','-p','enable_can:=false'])
        executor=StackCanExecutor();bridge=WebSocketTeleopBridge()
        assert not executor.enable_can and executor.can_bus is None
        assert bridge.drive_state==0 and executor.drive_state==0
        async def spin():
            while rclpy.ok():
                rclpy.spin_once(executor,timeout_sec=0);rclpy.spin_once(bridge,timeout_sec=0)
                await asyncio.sleep(.005)
        task=asyncio.create_task(spin())
        try:
            async with serve(bridge.handle_client,'127.0.0.1',19010):
                await asyncio.sleep(.5)
                async with connect('ws://127.0.0.1:19010') as ws:
                    await ws.send(json.dumps(dict(type='device',action='restart',request_id='integration-restart',confirm_stopped=True)))
                    async def heartbeat():
                        while True:
                            await ws.send(json.dumps(dict(type='heartbeat')));await asyncio.sleep(.3)
                    pulse=asyncio.create_task(heartbeat())
                    try:
                        first=json.loads(await asyncio.wait_for(ws.recv(),8))
                        assert first['code']=='RESTART_ACCEPTED' and first['startup_mode']=='paused'
                        second=json.loads(await asyncio.wait_for(ws.recv(),8))
                        assert second['code']=='RESTART_SCHEDULED'
                        await asyncio.gather(*controller.tasks)
                        restart.assert_awaited_once()
                        assert controller.marker.exists() and executor.drive_state in (0,2)
                        assert executor.can_maintenance.active
                        await ws.send(json.dumps(dict(type='device',action='status',request_id='query')))
                        status=json.loads(await asyncio.wait_for(ws.recv(),5))
                        assert status['job']['state']=='completed' and status['startup_pause_armed']
                        print('PASS: actual 9010 -> pause acknowledgement -> ACCEPTED -> host commit -> mocked restart',flush=True)
                        print('PASS: startup and restart gate remain paused; host status reports job without issuing another restart',flush=True)
                    finally:
                        pulse.cancel();await asyncio.gather(pulse,return_exceptions=True)
        finally:
            for pending in list(bridge.device_control.tasks):pending.cancel()
            await asyncio.gather(*bridge.device_control.tasks,return_exceptions=True)
            task.cancel();await asyncio.gather(task,return_exceptions=True)
            unix.close();await unix.wait_closed()
            bridge.disable_raw();bridge.destroy_node();executor.destroy_node();rclpy.shutdown()
asyncio.run(run())
