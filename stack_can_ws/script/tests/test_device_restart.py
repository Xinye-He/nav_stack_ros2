"""Mocked systemctl only: never restart real navigation in this test."""
import asyncio,importlib.util,json,sys,tempfile,time,unittest,os,subprocess
from pathlib import Path
from types import SimpleNamespace as O
from unittest.mock import AsyncMock,Mock,patch
ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('host_device',ROOT/'script/device_control/service.py');host=importlib.util.module_from_spec(spec);spec.loader.exec_module(host)
class HostTests(unittest.IsolatedAsyncioTestCase):
    def request(self,rid='r',action='prepare',token=None):
        return dict(request_id=rid,action=action,token=token,confirm_stopped=True,safety_proof=dict(request_id=rid,mode='executor_paused',checked_at=time.monotonic()))
    async def test_two_phase_marker_and_exactly_one_restart(self):
        with tempfile.TemporaryDirectory() as temp:
            restart=AsyncMock();c=host.Controller(temp,restart,delay=.01)
            prep=await c.execute(self.request());restart.assert_not_awaited();self.assertFalse(c.marker.exists())
            commit=self.request(action='commit',token=prep['token'])
            result=await c.execute(commit);self.assertEqual(result['code'],'RESTART_SCHEDULED');self.assertTrue(c.marker.exists())
            await c.execute(commit);await asyncio.gather(*c.tasks)
            restart.assert_awaited_once();self.assertEqual(c.jobs['r']['state'],'completed')
    async def test_invalid_confirmation_or_stale_proof_never_restarts(self):
        with tempfile.TemporaryDirectory() as temp:
            restart=AsyncMock();c=host.Controller(temp,restart)
            request=self.request();request['confirm_stopped']=False
            self.assertFalse((await c.execute(request))['ok'])
            request=self.request();request['safety_proof']['checked_at']-=3
            self.assertFalse((await c.execute(request))['ok']);restart.assert_not_awaited()
    async def test_expired_or_cancelled_prepare_cannot_commit(self):
        with tempfile.TemporaryDirectory() as temp:
            restart=AsyncMock();c=host.Controller(temp,restart)
            prep=await c.execute(self.request());c.jobs['r']['expires_at']=time.monotonic()-1
            self.assertFalse((await c.execute(self.request(action='commit',token=prep['token'])))['ok'])
            prep=await c.execute(self.request('new'))
            self.assertTrue((await c.execute(self.request('new','cancel',prep['token'])))['ok'])
            self.assertFalse((await c.execute(self.request('new','commit',prep['token'])))['ok']);restart.assert_not_awaited()
    def test_demo_startup_marker_forces_pause(self):
        script=(ROOT.parent/'docker/demo.sh').read_text()
        part=script[script.index('NAV_START_PAUSED='):script.index('# 避免容器名冲突')]
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'script/device_control/.runtime';path.mkdir(parents=True)
            (path/'start_paused.json').write_text('{}')
            result=subprocess.check_output(['bash','-c',part+'\nprintf %s \"$NAV_START_PAUSED\"'],env=dict(os.environ,STACK_CAN_WS_DIR=temp,NAV_START_PAUSED='0'))
            self.assertEqual(result,b'1')
        self.assertIn('-e NAV_START_PAUSED=',script)
    async def test_fixed_service_and_invalid_token(self):
        with tempfile.TemporaryDirectory() as temp:
            c=host.Controller(temp,AsyncMock());prep=await c.execute(self.request())
            self.assertFalse((await c.execute(self.request(action='commit',token='wrong')))['ok'])
        proc=O(returncode=0,communicate=AsyncMock(return_value=(b'',b'')))
        with patch.object(host.asyncio,'create_subprocess_exec',new=AsyncMock(return_value=proc)) as call:
            await host.restart_navigation()
            self.assertEqual(call.call_args.args,('/bin/systemctl','restart','nav-stack-ros2.service'))
    async def test_failure_survives_in_status_and_marker_stays_paused(self):
        with tempfile.TemporaryDirectory() as temp:
            c=host.Controller(temp,AsyncMock(side_effect=RuntimeError('failed')),delay=.001)
            prep=await c.execute(self.request());await c.execute(self.request(action='commit',token=prep['token']))
            await asyncio.gather(*c.tasks)
            result=await c.execute(dict(action='status',request_id='s'))
            self.assertEqual(result['job']['state'],'failed');self.assertTrue(result['startup_pause_armed'])
    async def test_daemon_restart_never_replays_pending_command(self):
        with tempfile.TemporaryDirectory() as temp:
            c=host.Controller(temp,AsyncMock());await c.execute(self.request())
            c.jobs['r']['state']='restarting';c.persist()
            restart=AsyncMock();again=host.Controller(temp,restart)
            self.assertEqual(again.jobs['r']['state'],'unknown');restart.assert_not_awaited()
            self.assertFalse((await again.execute(self.request('other')))['ok'])

sys.path.insert(0,str(ROOT/'src/main'));sys.path.insert(0,str(ROOT/'script/websocket'))
try:
    from main import device_control as client
    from main.can_management import CanControlClient
    from main.websocket_teleop_key import WebSocketTeleopBridge
    ROS=True
except ImportError:ROS=False

@unittest.skipUnless(ROS,'requires ROS modules')
class ClientTests(unittest.IsolatedAsyncioTestCase):
    def fixture(self):
        can=O(pending=False,maintenance_rid=None,zero_and_pause=Mock())
        def maintenance(rid,active):can.maintenance_rid=rid if active else None
        can.maintenance=Mock(side_effect=maintenance)
        async def proof(rid):can.maintenance(rid,True);return dict(request_id=rid,mode='executor_paused',checked_at=time.monotonic())
        can.safe_proof=AsyncMock(side_effect=proof)
        c=client.DeviceControlClient(O(),can);ws=O(send=AsyncMock());return c,can,ws
    async def test_confirmation_and_busy_rejection_have_no_restart(self):
        c,can,ws=self.fixture()
        await c.submit(ws,dict(action='restart',request_id='r'))
        self.assertEqual(json.loads(ws.send.call_args.args[0])['code'],'DEVICE_STOP_CONFIRMATION_REQUIRED')
        can.pending=True;await c.submit(ws,dict(action='restart',request_id='r',confirm_stopped=True))
        self.assertEqual(json.loads(ws.send.call_args.args[0])['code'],'DEVICE_BUSY');can.safe_proof.assert_not_awaited()
    async def test_ack_sent_before_commit_and_interlock_held(self):
        c,can,ws=self.fixture();order=[]
        async def send(raw):order.append(json.loads(raw)['code'])
        ws.send.side_effect=send
        async def rpc(req):
            order.append(req['action'])
            if req['action']=='prepare':return dict(ok=True,token='token',job=dict(state='prepared'))
            return dict(ok=True,code='RESTART_SCHEDULED')
        c.watch=AsyncMock(return_value=True)
        with patch.object(client,'host_request',side_effect=rpc):
            await c.submit(ws,dict(action='restart',request_id='r',confirm_stopped=True));await asyncio.gather(*c.tasks)
        self.assertLess(order.index('RESTART_ACCEPTED'),order.index('commit'))
        self.assertTrue(can.pending);self.assertEqual(can.maintenance_rid,'r')
    async def test_failed_websocket_ack_never_commits(self):
        c,can,ws=self.fixture();ws.send.side_effect=ConnectionError('closed');actions=[]
        async def rpc(req):
            actions.append(req['action']);return dict(ok=True,token='t',job=dict(state='prepared'))
        with patch.object(client,'host_request',side_effect=rpc):
            await c.submit(ws,dict(action='restart',request_id='r',confirm_stopped=True));await asyncio.gather(*c.tasks)
        self.assertNotIn('commit',actions);self.assertIn('cancel',actions);self.assertFalse(can.pending);self.assertIsNone(can.maintenance_rid)
    async def test_running_vehicle_rejection_never_reaches_host(self):
        c,can,ws=self.fixture();can.safe_proof=AsyncMock(side_effect=ValueError('车辆尚未暂停'))
        with patch.object(client,'host_request',new=AsyncMock()) as rpc:
            await c.submit(ws,dict(action='restart',request_id='r',confirm_stopped=True));await asyncio.gather(*c.tasks)
            rpc.assert_not_awaited()
        self.assertFalse(can.pending);self.assertEqual(json.loads(ws.send.call_args.args[0])['code'],'DEVICE_REQUEST_REJECTED')
    async def test_cli_status_is_not_a_control_or_heartbeat_connection(self):
        from contextlib import asynccontextmanager
        import hmi_keyboard_client as keyboard
        sent=[]
        class WS:
            async def send(self,raw):sent.append(json.loads(raw))
            async def recv(self):return json.dumps(dict(type='device_result',request_id=sent[0]['request_id'],action='status',ok=True,code='OK'))
        @asynccontextmanager
        async def connect(*args,**kwargs):yield WS()
        args=O(work_only=True,dry_run=False,upload_fence=None,upload_points=None,device_action='status',host='127.0.0.1',control_port=1,confirm_stopped=False)
        with patch.object(keyboard,'connect_api',return_value=connect):await keyboard.run(args)
        self.assertEqual(len(sent),1);self.assertEqual(sent[0]['type'],'device');self.assertEqual(sent[0]['action'],'status')
    async def test_cli_does_not_retry_when_connection_drops_after_acceptance(self):
        from contextlib import asynccontextmanager
        import hmi_keyboard_client as keyboard
        sent=[]
        class WS:
            count=0
            async def send(self,raw):sent.append(json.loads(raw))
            async def recv(self):
                self.count+=1
                if self.count==1:return json.dumps(dict(type='device_result',request_id=sent[0]['request_id'],ok=True,code='RESTART_ACCEPTED'))
                raise ConnectionError('navigation stopping')
        @asynccontextmanager
        async def connect(*args,**kwargs):yield WS()
        args=O(work_only=True,dry_run=False,upload_fence=None,upload_points=None,device_action='restart',host='127.0.0.1',control_port=1,confirm_stopped=True)
        with patch.object(keyboard,'connect_api',return_value=connect):await keyboard.run(args)
        self.assertEqual(len([m for m in sent if m.get('action')=='restart']),1)
    async def test_lost_commit_reply_keeps_pause(self):
        c,can,ws=self.fixture()
        async def rpc(req):
            if req['action']=='prepare':return dict(ok=True,token='t',job=dict(state='prepared'))
            raise OSError('lost commit reply')
        c.watch=AsyncMock(return_value=True)
        with patch.object(client,'host_request',side_effect=rpc):
            await c.submit(ws,dict(action='restart',request_id='r',confirm_stopped=True));await asyncio.gather(*c.tasks)
        self.assertTrue(can.pending);self.assertEqual(can.maintenance_rid,'r')
    async def test_status_is_read_only_and_failure_releases_pause(self):
        c,can,ws=self.fixture()
        with patch.object(client,'host_request',new=AsyncMock(return_value=dict(ok=True,code='OK'))):
            await c.submit(ws,dict(action='status',request_id='s'));await asyncio.gather(*c.tasks)
        can.safe_proof.assert_not_awaited();can.zero_and_pause.assert_not_called()
        async def rpc(req):return dict(ok=True,token='t',job=dict(state='prepared')) if req['action']=='prepare' else dict(ok=True,code='RESTART_SCHEDULED')
        c.watch=AsyncMock(return_value=False)
        with patch.object(client,'host_request',side_effect=rpc):
            await c.submit(ws,dict(action='restart',request_id='r',confirm_stopped=True));await asyncio.gather(*c.tasks)
        self.assertFalse(can.pending);self.assertIsNone(can.maintenance_rid)

@unittest.skipUnless(ROS,'requires ROS modules')
class StartupTests(unittest.TestCase):
    def test_bridge_starts_paused_when_restart_marker_exported(self):
        import rclpy
        rclpy.init(args=[])
        published=[]
        try:
            with patch.dict(os.environ,{'NAV_START_PAUSED':'1'}),patch.object(WebSocketTeleopBridge,'publish_drive_cmd',new=lambda self:published.append(self.drive_state)):
                node=WebSocketTeleopBridge()
                try:self.assertEqual(node.drive_state,0);self.assertEqual(published,[0])
                finally:node.disable_raw();node.destroy_node()
        finally:rclpy.shutdown()

if __name__=='__main__':unittest.main(verbosity=2)
