from pathlib import Path
import asyncio,json,sys,tempfile,time,unittest
from types import SimpleNamespace as Obj
from unittest.mock import Mock
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/main'));sys.path.insert(0,str(ROOT/'script/websocket'))
from main.trajectory_control import load_snapshot,ExecutorPathControl,FollowerPathControl,BridgePathClient
from std_msgs.msg import String
from hmi_keyboard_client import KeyboardCommands

def zero():return Obj(pre_speed_kmh=0.,angle_deg=0.,pick=False,unload=False,dump=False,pick_action=False,valid=True)
def message(action,rid='r1',version=None):
    return String(data=json.dumps(dict(action=action,request_id=rid,path_version=version)))
class Node:
    DS_PAUSED=0;DS_RUNNING=1;DS_ESTOP=2
    def __init__(self):
        self.pubs={};self.remote_req=False;self.abort_req=False;self.estop_latched=False;self.estop_reason=''
        self.drive_state=0;self.teleop_active=True;self.bale_active=False;self.cmd_teleop=zero();self.cmd_traj=zero()
        self.paused_reset_guards={k:(True,time.monotonic()) for k in ('围栏','9010链路','外设及控制节点')}
    def create_publisher(self,kind,topic,depth):
        self.pubs[topic]=Mock();return self.pubs[topic]
    def create_subscription(self,*args):pass
    def create_timer(self,*args):pass
    def _make_stop_command(self):return zero()
    def _is_zero_cmd(self,msg):return msg.pre_speed_kmh==0 and msg.angle_deg==0 and not any((msg.pick,msg.unload,msg.dump,msg.pick_action))
    def _update_safety_state(self):self.safety_state='READY'
    def _update_control_mode(self):self.control_mode='TRAJ'
class Geo:
    def __init__(self,*args):pass
    def lla_to_enu(self,lat,lon,alt):return lat,lon,alt
class Tests(unittest.TestCase):
    def test_snapshot_validation_and_version(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'points.csv';p.write_text('idx,lat,lon,heading,type,extra\n0,29,119,90,0,0\n')
            rows,version=load_snapshot(p);self.assertEqual(len(rows),1);self.assertEqual(len(version),64)
            p.write_text('0,nan,119,90,0,0\n')
            with self.assertRaises(ValueError):load_snapshot(p)
            p.write_text('0,29,119,90\n0,29,119,90\n')
            with self.assertRaises(ValueError):load_snapshot(p)
    def test_reload_enters_zero_teleop_without_starting_follower(self):
        node=Node();ctl=ExecutorPathControl(node)
        ctl.handle(message('reload'))
        self.assertEqual(node.drive_state,0);self.assertTrue(node.teleop_active)
        self.assertIsNone(node.cmd_traj)
        ctl.on_result(String(data=json.dumps(dict(request_id='r1',action='reload',ok=True,path_version='v1',waypoint_count=2))))
        result=json.loads(node.pubs['/vehicle/trajectory_result'].publish.call_args.args[0].data)
        self.assertTrue(result['ok']);self.assertEqual(result['drive_state'],1)
        self.assertTrue(node.teleop_active);self.assertTrue(node._is_zero_cmd(node.cmd_teleop))
        node.pubs['/drive_cmd'].publish.assert_not_called()
    def test_start_needs_reload_and_all_guards(self):
        node=Node();ctl=ExecutorPathControl(node)
        ctl.handle(message('start',version='v1'));self.assertIsNone(ctl.pending)
        ctl.loaded_version='v1';ctl.traj_received=time.monotonic()
        node.estop_latched=True
        ctl.handle(message('start',version='v1'));node.pubs['/drive_cmd'].publish.assert_not_called()
        node.estop_latched=False;node.paused_reset_guards['围栏']=(False,time.monotonic())
        ctl.handle(message('start',version='v1'));node.pubs['/drive_cmd'].publish.assert_not_called()
    def test_start_two_phase_and_rechecks_estop(self):
        for interrupted in (False,True):
            node=Node();ctl=ExecutorPathControl(node);ctl.loaded_version='v1';ctl.traj_received=time.monotonic()
            ctl.handle(message('start','s1','v1'))
            self.assertEqual(node.drive_state,0);node.pubs['/drive_cmd'].publish.assert_not_called()
            node.estop_latched=interrupted
            ctl.on_result(String(data=json.dumps(dict(request_id='s1',action='start',ok=True,waypoint_count=2))))
            if interrupted:node.pubs['/drive_cmd'].publish.assert_not_called()
            else:
                self.assertEqual(node.drive_state,1);self.assertFalse(node.teleop_active)
                self.assertEqual(node.pubs['/drive_cmd'].publish.call_args.args[0].data,1)
    def test_timeout_keeps_paused(self):
        node=Node();ctl=ExecutorPathControl(node);ctl.handle(message('reload'))
        ctl.pending=(ctl.pending[0],time.monotonic()-9);ctl.check_timeout()
        self.assertIsNone(ctl.pending);self.assertEqual(node.drive_state,0)
        node.pubs['/drive_cmd'].publish.assert_not_called()
    def test_follower_reload_is_transactional_and_start_only_prepares(self):
        with tempfile.TemporaryDirectory() as tmp:
            node=Node();node.path_csv=str(Path(tmp)/'points.csv');node.map_frame='map'
            node.publish_traj_cmd=Mock();node._pub_waiting=Mock();node._pub_global_path=Mock()
            node.completed_unload_indices=set();node.get_parameter=lambda name:Obj(value=0)
            node.get_clock=lambda:Obj(now=lambda:0)
            Path(node.path_csv).write_text('0,29,119,90,0,0\n1,29.1,119.1,90,0,0\n')
            ctl=FollowerPathControl(node,Geo,lambda value:value)
            ctl.handle(message('reload'));self.assertEqual(node.drive_state,0)
            version=ctl.version;old=node.waypoints_xy
            node.cur_x=node.cur_y=node.cur_yaw=0
            ctl.position_received=ctl.heading_received=time.monotonic()
            ctl.handle(message('start','s1',version));self.assertEqual(node.drive_state,0)
            self.assertTrue(json.loads(ctl.pub.publish.call_args.args[0].data)['ok'])
            Path(node.path_csv).write_text('broken\n')
            ctl.handle(message('reload','r2'));self.assertIs(node.waypoints_xy,old)
            self.assertIsNone(ctl.version);self.assertEqual(node.drive_state,0)
    def test_keyboard_split_and_work_mode_blocks(self):
        keys=KeyboardCommands(work_only=False)
        with __import__('contextlib').redirect_stdout(__import__('io').StringIO()):self.assertIsNone(keys.message('y'))
        self.assertEqual(keys.message('n')['action'],'reload');keys.path_version='v1'
        self.assertEqual(keys.message('t')['action'],'toggle')
        self.assertIsNone(keys.message('y'))
        work=KeyboardCommands(work_only=True)
        with __import__('contextlib').redirect_stdout(__import__('io').StringIO()):
            self.assertIsNone(work.message('n'));self.assertIsNone(work.message('t'));self.assertIsNone(work.message('y'))
    def test_remote_switch_zero_and_pause_follower(self):
        node=Node();node.teleop_active=False;node.drive_state=1
        ctl=ExecutorPathControl(node);ctl.loaded_version='v1'
        ctl.handle(message('remote'));self.assertEqual(node.drive_state,0)
        ctl.on_result(String(data=json.dumps(dict(request_id='r1',action='remote',ok=True))))
        self.assertTrue(node.teleop_active);self.assertEqual(node.drive_state,1)
        self.assertTrue(node._is_zero_cmd(node.cmd_teleop));self.assertEqual(ctl.loaded_version,'v1')
        node.pubs['/drive_cmd'].publish.assert_not_called()
    def test_reload_does_not_clear_estop(self):
        node=Node();node.estop_latched=True;node.abort_req=True
        ctl=ExecutorPathControl(node);ctl.handle(message('reload'))
        ctl.on_result(String(data=json.dumps(dict(request_id='r1',action='reload',ok=True,path_version='v1',waypoint_count=2))))
        self.assertEqual(node.drive_state,2);self.assertTrue(node.estop_latched)
        node.pubs['/drive_cmd'].publish.assert_not_called()
    def test_bridge_toggle_uses_loaded_version_and_current_mode(self):
        async def check(active, expected):
            node=Node();node.teleop_active=active;node.zero_teleop=Mock()
            ctl=BridgePathClient(node);ctl.loaded_version='v1'
            sent=[]
            class WS:
                async def send(self, raw):sent.append(json.loads(raw))
            def answer(msg):
                req=json.loads(msg.data)
                self.assertEqual(req['action'],expected)
                if expected=='start':self.assertEqual(req['path_version'],'v1')
                ctl.on_result(String(data=json.dumps(dict(type='trajectory_result',request_id=req['request_id'],action=expected,ok=True,drive_state=1))))
            ctl.pub.publish.side_effect=answer
            await ctl.request(WS(),dict(action='toggle',request_id='toggle1'))
            self.assertEqual(sent[0]['action'],expected)
            self.assertEqual(node.teleop_active,expected!='start')
            node.zero_teleop.assert_called_once()
        asyncio.run(check(True,'start'))
        asyncio.run(check(False,'remote'))
if __name__=='__main__':unittest.main(verbosity=2)
