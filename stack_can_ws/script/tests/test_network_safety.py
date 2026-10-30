"""No CAN devices: mode policy, session ownership and paused manual reset."""
import ast,json,sys,time,unittest
from pathlib import Path
from types import SimpleNamespace as O
from unittest.mock import Mock
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/main'));sys.path.insert(0,str(ROOT/'script'));sys.path.insert(0,str(ROOT/'script/websocket'));sys.path.insert(0,str(Path(__file__).parent))
tree=ast.parse((ROOT/'src/main/main/network_safety.py').read_text());ns={}
exec(compile(ast.Module(body=[n for n in tree.body if (isinstance(n,ast.FunctionDef) and n.name=='fault_action') or (isinstance(n,ast.ClassDef) and n.name=='ControlSessions')],type_ignores=[]),'network_policy','exec'),ns)
fault_action=ns['fault_action'];Sessions=ns['ControlSessions']
from network_status import summarize_network
class PolicyTests(unittest.TestCase):
    def node(self):
        return O(estop_latched=False,abort_req=False,drive_state=1,DS_ESTOP=2,DS_PAUSED=0,remote_req=False,
                 teleop_active=False,bale_active=False,cmd_traj=O(valid=True),cmd_teleop=None,cmd_bale=None,teleop_source='websocket_9010')
    def test_trajectory_warns_and_remote_teleop_estops(self):
        n=self.node();self.assertEqual(fault_action(n)[0],'warning')
        n.teleop_active=True;n.cmd_teleop=O(valid=True)
        self.assertEqual(fault_action(n)[0],'estop')
        n.drive_state=0;self.assertEqual(fault_action(n)[0],'estop')
    def test_unknown_running_fails_closed_local_and_existing_protection_unchanged(self):
        n=self.node();n.cmd_traj=None;self.assertEqual(fault_action(n)[0],'estop')
        n.teleop_active=True;n.cmd_teleop=O(valid=True);n.teleop_source='keyboard'
        self.assertEqual(fault_action(n)[0],'warning')
        n.abort_req=True;self.assertEqual(fault_action(n)[0],'warning')
    def test_query_sessions_cannot_interrupt_or_mask_owner(self):
        s=Sessions();s.connect('a',0);s.connect('query',0)
        self.assertTrue(s.message('a',0,heartbeat=True))
        self.assertFalse(s.message('query',10))
        self.assertEqual(s.clients['a'],0);self.assertFalse(s.disconnect('query'))
        self.assertTrue(s.disconnect('a'))
    def test_reconnect_and_late_old_disconnect(self):
        s=Sessions();s.connect('a',0);s.message('a',0,heartbeat=True);s.connect('b',2)
        self.assertTrue(s.message('b',2,heartbeat=True));self.assertFalse(s.disconnect('a'))
        self.assertEqual(s.owner,'b')
    def test_network_warning_log_is_not_an_abort_trigger(self):
        from status_diagnostics import DiagnosticEvents
        d=DiagnosticEvents();d.record(O(name='websocket_teleop_bridge',msg='NETWORK WARNING: trajectory continues',level=30,stamp=O(sec=1,nanosec=0)))
        self.assertEqual(d.events[0]['code'],'NETWORK_WARNING');self.assertEqual(d.events[0]['evidence'],'related_log')
    def test_network_status_reconnect_does_not_clear_estop(self):
        c=dict(safety_state='ESTOP',estop_reason='network_disconnected')
        self.assertEqual(summarize_network(c,True,True,True,None)['severity'],'emergency')
        self.assertEqual(summarize_network({},True,False,True,dict(message='循迹继续'))['severity'],'warning')
        self.assertEqual(summarize_network({},True,True,True,{})['severity'],'none')

try:
    from main.network_safety import ExecutorNetworkSafety
    from main.stack_can_executor import StackCanExecutor
    from main.websocket_teleop_key import WebSocketTeleopBridge
    from std_msgs.msg import String,Bool
    from test_paused_reset import Harness
    ROS=True
except ImportError:
    ROS=False

@unittest.skipUnless(ROS,'requires ROS message modules')
class RosTests(unittest.TestCase):
    def node(self,trajectory=True):
        n=Harness();n.drive_state=1;n.estop_latched=False;n.abort_req=False;n.estop_reason=''
        n.cmd_traj=n._make_stop_command();n.cmd_traj.pre_speed_kmh=4.
        n.teleop_active=not trajectory;n.teleop_source='websocket_9010'
        n.cmd_teleop=n._make_stop_command();n.cmd_teleop.pre_speed_kmh=4.
        n.create_subscription=lambda *args:None
        n.create_publisher=lambda *args:Mock()
        n.network_safety=ExecutorNetworkSafety(n)
        return n
    def event(self,n,rid='fault1',reason='network_disconnected'):
        n.network_safety.on_event(String(data=json.dumps(dict(action='fault',request_id=rid,reason_code=reason,detail='test disconnect'))))
        return json.loads(n.network_safety.pub.publish.call_args.args[0].data)
    def test_trajectory_continues_without_zeroing_or_abort(self):
        n=self.node(True);old=n.cmd_traj
        self.assertEqual(self.event(n)['action'],'warning')
        self.assertEqual(n.drive_state,1);self.assertFalse(n.estop_latched);self.assertIs(n.cmd_traj,old)
        n.loop();self.assertEqual(n.send_can.call_args.kwargs['speed_kmh'],4)
        self.assertTrue(n.send_can.call_args.kwargs['drive'])
    def test_remote_disconnect_and_manual_reset_stays_paused(self):
        n=self.node(False);self.assertEqual(self.event(n)['action'],'estop')
        self.assertTrue(n.estop_latched);self.assertEqual(n.drive_state,2);self.assertFalse(n.abort_req)
        self.assertEqual(n.estop_reason,'network_disconnected')
        result=n.request();self.assertTrue(result['ok']);self.assertEqual(result['cleared_network_event_id'],'fault1')
        self.assertEqual(n.drive_state,0);self.assertFalse(n.network_safety.active)
        n.loop();self.assertFalse(n.send_can.call_args.kwargs['drive'])
    def test_other_abort_cannot_be_cleared_as_network(self):
        n=self.node(False);self.event(n)
        StackCanExecutor._on_abort(n,Bool(data=True))
        self.assertEqual(n.estop_reason,'abort');self.assertFalse(n.request()['ok']);self.assertTrue(n.estop_latched)
    def test_link_or_guard_not_ready_rejects_reset(self):
        for label in ('围栏','9010链路','外设及控制节点'):
            n=self.node(False);self.event(n);n.paused_reset_guards[label]=(False,time.monotonic())
            self.assertFalse(n.request()['ok']);self.assertTrue(n.estop_latched)
    def test_legacy_run_reset_cannot_override_network_lock(self):
        n=self.node(False);self.event(n);n.reset_estop_req=True
        n._try_clear_estop();self.assertTrue(n.estop_latched);self.assertEqual(n.drive_state,2)
    def test_repeat_fault_retains_first_reason_and_id(self):
        n=self.node(False);self.event(n,'first','network_heartbeat_timeout');self.event(n,'second','network_disconnected')
        self.assertEqual(n.estop_reason,'network_heartbeat_timeout');self.assertEqual(n.network_safety.fault_id,'first')
    def test_bridge_warning_does_not_stop_trajectory_and_estop_ack_pauses(self):
        class Bridge:
            trigger_network_abort=WebSocketTeleopBridge.trigger_network_abort
            on_network_result=WebSocketTeleopBridge.on_network_result
        for trajectory in (True,False):
            b=Bridge();b.network_fault_latched=False;b.network_warning_latched=False;b.network_event_pending=None;b.ws_connected=False
            b.pub_network_event=Mock();b.publish_link_ok=Mock();b.publish_fault_text=Mock()
            b.zero_teleop=Mock();b.publish_teleop_cmd=Mock();b.publish_drive_cmd=Mock()
            n=self.node(trajectory)
            n.network_safety.pub.publish.side_effect=b.on_network_result
            b.pub_network_event.publish.side_effect=n.network_safety.on_event
            b.trigger_network_abort('websocket disconnected: test')
            if trajectory:
                b.zero_teleop.assert_not_called();b.publish_drive_cmd.assert_not_called();self.assertFalse(b.network_fault_latched)
            else:
                self.assertTrue(b.network_fault_latched);self.assertEqual(b.drive_state,0)
            self.assertIsNone(b.network_event_pending)

if __name__=='__main__':unittest.main(verbosity=2)
