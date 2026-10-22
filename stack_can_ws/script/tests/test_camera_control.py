"""Camera orchestration tests. Never start real cameras or Docker containers."""
import asyncio
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import AsyncMock, patch

ROOT = Path(__file__).resolve().parents[2]
def load(name, path):
    spec=importlib.util.spec_from_file_location(name, path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
host=load('camera_host', ROOT/'script/video_usb/camera_control_service.py')
client_module=load('camera_client', ROOT/'src/main/main/camera_control.py')

class CameraTests(unittest.IsolatedAsyncioTestCase):
    async def test_invalid_action_and_request_id(self):
        controller=host.CameraController('jetson')
        response=await controller.execute({'action':'shell', 'request_id':'1'})
        self.assertEqual(response['code'], 'INVALID_REQUEST')
        self.assertFalse(response['ok'])

    async def test_start_stop_and_duplicate_request(self):
        controller=host.CameraController('jetson')
        stopped={'camera_state':'stopped','cameras':[{'name':'cam0','device':'/dev/null','device_present':True,'stream_ready':False}]}
        running={'camera_state':'running','cameras':[{'name':'cam0','device':'/dev/null','device_present':True,'stream_ready':True}]}
        controller.status=AsyncMock(side_effect=[stopped, running, running, stopped])
        with patch.object(host,'command',AsyncMock(return_value=(0,'done'))) as run:
            response=await controller.execute({'action':'start','request_id':'start1'})
            self.assertTrue(response['ok'])
            duplicate=await controller.execute({'action':'start','request_id':'start1'})
            self.assertEqual(duplicate,response)
            self.assertEqual(run.await_count,1)
            conflict=await controller.execute({'action':'stop','request_id':'start1'})
            self.assertEqual(conflict['code'],'REQUEST_ID_CONFLICT')
            response=await controller.execute({'action':'stop','request_id':'stop1'})
            self.assertTrue(response['ok'])
            self.assertEqual(response['camera_state'],'stopped')

    async def test_missing_camera_does_not_run_start_script(self):
        controller=host.CameraController('jetson')
        controller.status=AsyncMock(return_value={'camera_state':'stopped', 'cameras':[{'device':'/dev/not-a-camera-test'}]})
        with patch.object(host,'command',AsyncMock()) as run:
            response=await controller.execute({'action':'start','request_id':'1'})
            self.assertEqual(response['code'],'CAMERA_DEVICE_MISSING')
            run.assert_not_awaited()

    async def test_script_failure_and_busy(self):
        controller=host.CameraController('jetson')
        controller.status=AsyncMock(return_value={'camera_state':'partial','cameras':[]})
        with patch.object(host,'command',AsyncMock(return_value=(2,'failed'))):
            response=await controller.execute({'action':'stop','request_id':'1'})
            self.assertFalse(response['ok'])
            self.assertEqual(response['exit_code'],2)
        controller.busy=True
        self.assertEqual((await controller.execute({'action':'start','request_id':'2'}))['code'],'CAMERA_BUSY')

    async def test_unix_proxy_is_nonblocking_and_returns_result(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'control.sock')
            async def agent(reader,writer):
                request=json.loads(await reader.readline())
                await asyncio.sleep(0.1)
                writer.write((json.dumps(dict(type='camera_result',ok=True,request_id=request['request_id'],camera_state='running'))+'\n').encode())
                await writer.drain()
                writer.close()
                await writer.wait_closed()
            server=await asyncio.start_unix_server(agent,path=path)
            ws=type('Socket',(),{})()
            ws.send=AsyncMock()
            client=client_module.CameraControlClient()
            try:
                with patch.dict(os.environ,{'CAMERA_CONTROL_SOCKET':path}):
                    await client.submit(ws,{'action':'start','request_id':'start1'})
                    self.assertTrue(client.pending)
                    # The receive loop can keep processing heartbeat messages immediately.
                    self.assertFalse(ws.send.called)
                    await client.submit(ws,{'action':'stop','request_id':'stop1'})
                    self.assertEqual(json.loads(ws.send.call_args.args[0])['code'],'CAMERA_BUSY')
                    await asyncio.gather(*list(client.tasks))
                    self.assertEqual(json.loads(ws.send.call_args.args[0])['camera_state'],'running')
                    self.assertFalse(client.pending)
            finally:
                server.close();await server.wait_closed()

    async def test_service_unavailable_returns_error(self):
        ws=type('Socket',(),{})();ws.send=AsyncMock()
        client=client_module.CameraControlClient()
        with patch.dict(os.environ,{'CAMERA_CONTROL_SOCKET':'/tmp/camera-test-no-socket'}):
            await client.submit(ws,{'action':'status','request_id':'1'})
            await asyncio.gather(*list(client.tasks))
        self.assertEqual(json.loads(ws.send.call_args.args[0])['code'],'CAMERA_SERVICE_UNAVAILABLE')

if __name__=='__main__':
    unittest.main(verbosity=2)
