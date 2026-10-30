"""No real CAN commands: host runner and ROS publishers are mocked."""
import asyncio,importlib.util,json,os,sys,tempfile,time,unittest
from pathlib import Path
from types import SimpleNamespace as O
from unittest.mock import Mock,AsyncMock,patch
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'script'));sys.path.insert(0,str(ROOT/'src/main'));sys.path.insert(0,str(ROOT/'script/websocket'))
spec=importlib.util.spec_from_file_location('host_can',ROOT/'script/can_control/service.py');host=importlib.util.module_from_spec(spec);spec.loader.exec_module(host)
from can_bus_status import enrich_can_status

class HostTests(unittest.IsolatedAsyncioTestCase):
    def proof(self,rid='r1'):
        return dict(action='recover',request_id=rid,confirm_stopped=True,safety_proof=dict(mode='executor_paused',request_id=rid,checked_at=time.monotonic()))
    async def test_busoff_single_recovery_and_duplicate_id(self):
        probe=AsyncMock(side_effect=[dict(state='BUS-OFF',interface_up=True),dict(state='ERROR-ACTIVE',interface_up=True)])
        restart=AsyncMock();c=host.Controller(probe,restart)
        a=await c.execute(self.proof());b=await c.execute(self.proof())
        self.assertTrue(a['ok']);self.assertEqual(a,b);restart.assert_awaited_once()
    async def test_reject_stale_proof_and_missing_confirmation(self):
        restart=AsyncMock();c=host.Controller(AsyncMock(),restart)
        request=self.proof();request['safety_proof']['checked_at']-=3
        self.assertFalse((await c.execute(request))['ok'])
        request=self.proof();request['confirm_stopped']=False
        self.assertFalse((await c.execute(request))['ok']);restart.assert_not_awaited()
    async def test_never_changes_bitrate_or_down_interface(self):
        for state,up in [('MISSING',False),('STOPPED',False),('ERROR-PASSIVE',True),('ERROR-ACTIVE',True)]:
            restart=AsyncMock();c=host.Controller(AsyncMock(return_value=dict(state=state,interface_up=up)),restart)
            result=await c.execute(self.proof());restart.assert_not_awaited()
            self.assertEqual(result['ok'],state=='ERROR-ACTIVE')
        c=host.Controller()
        with patch.object(host,'command',new=AsyncMock()) as command:
            await c.restart_bus()
            self.assertEqual(command.call_args.args[1:],('link','set','dev','can0','type','can','restart'))
    async def test_busy_query_and_failed_restart(self):
        c=host.Controller(AsyncMock(return_value=dict(state='BUS-OFF',interface_up=True)),AsyncMock(side_effect=RuntimeError('failure')))
        c.busy=True
        self.assertEqual((await c.execute(self.proof()))['code'],'CAN_BUSY')
        self.assertTrue((await c.execute(dict(action='status',request_id='s')))['bus']['recovering'])
        c.busy=False;self.assertFalse((await c.execute(self.proof()))['ok']);self.assertFalse(c.busy)
    async def test_busoff_again_is_failure(self):
        c=host.Controller(AsyncMock(return_value=dict(state='BUS-OFF',interface_up=True)),AsyncMock())
        self.assertEqual((await c.execute(self.proof()))['code'],'CAN_INTERFACE_NOT_RECOVERED')
    def test_decode_link_and_enrichment(self):
        bus=host.decode_link([dict(flags=['UP'],operstate='DOWN',linkinfo=dict(info_data=dict(state='BUS-OFF',berr_counter=dict(tx=255,rx=127),bittiming=dict(bitrate=250000),restart_ms=0)),stats64=dict(rx=dict(packets=10),tx=dict(packets=20)))])
        self.assertEqual(bus['bitrate'],250000);self.assertEqual(bus['rx_packets'],10)
        result=enrich_can_status(dict(state='ready',errors=[]),bus,10,10)
        self.assertEqual(result['state'],'error');self.assertEqual(result['errors'][0]['code'],'CAN_BUS_OFF')
        self.assertEqual(result['bus']['tx_error_counter'],255)
    def test_status_timeout_down_missing_and_healthy(self):
        item=dict(state='ready',errors=[])
        self.assertEqual(enrich_can_status(item,None,None,10)['errors'][0]['code'],'CAN_BUS_STATUS_TIMEOUT')
        self.assertEqual(enrich_can_status(item,None,None,10,True)['state'],'starting')
        for state,up,code in [('MISSING',False,'CAN_INTERFACE_MISSING'),('STOPPED',False,'CAN_INTERFACE_DOWN')]:
            self.assertEqual(enrich_can_status(item,dict(state=state,interface_up=up),10,10)['errors'][0]['code'],code)
        self.assertEqual(enrich_can_status(item,dict(state='ERROR-ACTIVE',interface_up=True),10,10)['state'],'ready')
        self.assertEqual(item,dict(state='ready',errors=[]))

# Run the full module in ROS; these imports never open a CAN bus.
try:
    from main import can_management as ros
    from std_msgs.msg import String,UInt8
    from main.stack_can_executor import StackCanExecutor
    from main.trajectory_control import ExecutorPathControl
    from hmi_keyboard_client import KeyboardCommands
except ImportError:
    ros=None

class FakeNode:
    DS_PAUSED=0;DS_ESTOP=2;DS_RUNNING=1
    def __init__(self):
        self.pubs={};self.estop_latched=False;self.abort_req=False;self.remote_req=False
        self.drive_state=0;self.reset_estop_req=False;self.teleop_active=True;self.bale_active=False
        self.zero_teleop=Mock();self.publish_drive_cmd=Mock();self.publish_teleop_active=Mock();self.publish_teleop_cmd=Mock()
        self.get_node_names=lambda:['stack_can_executor']
    def create_subscription(self,*args):pass
    def create_timer(self,*args):pass
    def create_publisher(self,kind,topic,qos):
        self.pubs[topic]=Mock();return self.pubs[topic]
    def _make_stop_command(self):
        return O(valid=True,pre_speed_kmh=0.,angle_deg=0.,pick=False,unload=False,dump=False,pick_action=False)

@unittest.skipIf(ros is None,'ROS modules not available')
class RosTests(unittest.IsolatedAsyncioTestCase):
    def ready_client(self):
        node=FakeNode();client=ros.CanControlClient(node)
        client.execution=dict(drive_state=0,remote_req=False);client.execution_at=time.monotonic()
        return node,client
    async def test_status_has_no_pause_or_recovery_side_effect(self):
        node,client=self.ready_client();ws=O(send=AsyncMock())
        with patch.object(ros,'host_request',new=AsyncMock(return_value=dict(type='can_result',action='status',request_id='s',ok=True))):
            await client.submit(ws,dict(action='status',request_id='s'))
            await asyncio.gather(*client.tasks)
        node.publish_drive_cmd.assert_not_called();self.assertFalse(client.pending)
    async def test_confirm_stopped_required_and_running_rejected(self):
        node,client=self.ready_client();ws=O(send=AsyncMock())
        await client.submit(ws,dict(action='recover',request_id='r'))
        self.assertEqual(json.loads(ws.send.call_args.args[0])['code'],'CAN_STOP_CONFIRMATION_REQUIRED')
        client.execution['drive_state']=1
        with patch.object(ros,'host_request',new=AsyncMock()) as host_rpc:
            await client.submit(ws,dict(action='recover',request_id='r',confirm_stopped=True))
            await asyncio.gather(*client.tasks)
            host_rpc.assert_not_awaited()
        node.publish_drive_cmd.assert_not_called();self.assertFalse(client.pending)
    async def test_recovery_ack_feedback_and_no_run_or_reset(self):
        node,client=self.ready_client();ws=O(send=AsyncMock())
        def ack(msg):
            req=json.loads(msg.data)
            if req['active']:
                client.on_execution(String(data=json.dumps(dict(drive_state=0,can_maintenance_active=True,can_maintenance_request_id=req['request_id']))))
        client.pub.publish.side_effect=ack
        async def rpc(req):
            if req['action']=='recover':
                self.assertEqual(req['safety_proof']['mode'],'executor_paused')
                async def feedback():
                    await asyncio.sleep(.03)
                    client.on_feedback(O(rpm_left_valid=True,rpm_right_valid=False,throttle_left_valid=False,throttle_right_valid=False,ground_speed_mps=0))
                asyncio.create_task(feedback())
            return dict(type='can_result',request_id=req['request_id'],action=req['action'],ok=True,bus=dict(state='ERROR-ACTIVE',interface_up=True))
        with patch.object(ros,'host_request',side_effect=rpc):
            await client.submit(ws,dict(action='recover',request_id='r',confirm_stopped=True))
            await asyncio.gather(*client.tasks)
        result=json.loads(ws.send.call_args.args[0]);self.assertTrue(result['ok']);self.assertTrue(result['feedback_valid'])
        self.assertEqual(node.drive_state,0);self.assertFalse(client.pending)
        self.assertEqual([json.loads(c.args[0].data)['active'] for c in client.pub.publish.call_args_list],[True,False])
    async def test_host_failure_still_releases_to_pause(self):
        node,client=self.ready_client();ws=O(send=AsyncMock())
        async def proof(rid):client.maintenance(rid,True);return dict(mode='executor_paused',request_id=rid,checked_at=time.monotonic())
        client.safe_proof=proof
        with patch.object(ros,'host_request',side_effect=OSError('offline')):
            await client.submit(ws,dict(action='recover',request_id='r',confirm_stopped=True));await asyncio.gather(*client.tasks)
        self.assertEqual(json.loads(ws.send.call_args.args[0])['code'],'CAN_SERVICE_UNAVAILABLE')
        self.assertEqual(node.drive_state,0);self.assertFalse(client.pending);self.assertIsNone(client.maintenance_rid)
    async def test_interface_recovered_but_feedback_missing_is_not_success(self):
        node,client=self.ready_client();ws=O(send=AsyncMock())
        async def proof(rid):client.maintenance(rid,True);return dict(mode='executor_paused',request_id=rid,checked_at=time.monotonic())
        client.safe_proof=proof
        async def rpc(req):return dict(type='can_result',action=req['action'],request_id=req['request_id'],ok=True,bus=dict(state='ERROR-ACTIVE',interface_up=True))
        with patch.object(ros,'host_request',side_effect=rpc):
            await client.submit(ws,dict(action='recover',request_id='r',confirm_stopped=True));await asyncio.gather(*client.tasks)
        reply=json.loads(ws.send.call_args.args[0]);self.assertFalse(reply['ok']);self.assertEqual(reply['code'],'CAN_FEEDBACK_NOT_RECOVERED')
        self.assertEqual(node.drive_state,0)
    def test_output_is_zero_and_jobs_off_while_maintenance_active(self):
        sys.path.insert(0,str(Path(__file__).resolve().parent))
        from test_paused_reset import Harness
        node=Harness();node.create_subscription=lambda *args:None
        node.estop_latched=False;node.abort_req=False;node.drive_state=1
        node.can_maintenance=ros.ExecutorCanMaintenance(node)
        node.can_maintenance.on_request(String(data=json.dumps(dict(request_id='r',active=True))))
        node.cmd_teleop.pre_speed_kmh=8.;node.cmd_teleop.dump=True;node.drive_state=1
        node.loop();out=node.send_can.call_args.kwargs
        self.assertEqual(out['speed_kmh'],0);self.assertFalse(out['drive']);self.assertFalse(out['dump'])

    async def test_missing_executor_only_allowed_on_startup_failure(self):
        node,client=self.ready_client();node.get_node_names=lambda:[];client.execution_at=None
        with tempfile.TemporaryDirectory() as temp:
            file=Path(temp)/'status.json';file.write_text(json.dumps(dict(device_state='startup_failed',ever_ready=False)))
            with patch.dict(os.environ,{'NAV_STATUS_FILE':str(file)}):
                proof=await client.safe_proof('r')
        self.assertEqual(proof['mode'],'startup_failed_no_executor');self.assertEqual(node.drive_state,0)
    def test_interlock_retains_estop_and_blocks_run_and_path(self):
        node=FakeNode();ctl=ros.ExecutorCanMaintenance(node);node.can_maintenance=ctl;node.path_control=O(pending=None)
        node.estop_latched=True;node.abort_req=True
        ctl.on_request(String(data=json.dumps(dict(request_id='r',active=True))))
        self.assertTrue(ctl.active);self.assertEqual(node.drive_state,2);self.assertTrue(node.estop_latched)
        StackCanExecutor._on_drive_cmd(node,UInt8(data=1));self.assertEqual(node.drive_state,2)
        node.reset_estop_req=True;StackCanExecutor._try_clear_estop(node);self.assertFalse(node.reset_estop_req)
        paths=ExecutorPathControl(node)
        paths.handle(String(data=json.dumps(dict(action='reload',request_id='p'))))
        self.assertEqual(json.loads(node.pubs['/vehicle/trajectory_result'].publish.call_args.args[0].data)['code'],'CAN_BUSY')
        ctl.on_request(String(data=json.dumps(dict(request_id='wrong',active=False))));self.assertTrue(ctl.active)
        ctl.on_request(String(data=json.dumps(dict(request_id='r',active=False))));self.assertFalse(ctl.active);self.assertEqual(node.drive_state,2)
    def test_moving_remote_and_keyboard(self):
        now=time.monotonic()
        self.assertIn('实体遥控',ros.recovery_rejection(dict(remote_req=True,drive_state=0),now,None,None,now))
        self.assertIn('移动',ros.recovery_rejection({},None,O(rpm_left_valid=True,rpm_right_valid=False,ground_speed_mps=.2),now,now))
        self.assertIn('移动',ros.recovery_rejection({},None,O(rpm_left_valid=True,rpm_right_valid=True,ground_speed_mps=0,left_wheel_speed_mps=.3,right_wheel_speed_mps=-.3),now,now))
        for work_only in (True,False):
            keys=KeyboardCommands(work_only=work_only)
            self.assertEqual(keys.message('o')['action'],'status');self.assertTrue(keys.message('i')['confirm_stopped'])

if __name__=='__main__':unittest.main(verbosity=2)
