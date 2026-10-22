import json,sys,time,unittest
from pathlib import Path
from types import SimpleNamespace as Obj
from unittest.mock import Mock
r=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(r/'src/main'))
sys.path.insert(0,str(r/'script/websocket'))
from main.paused_reset import rejection
from main.stack_can_executor import StackCanExecutor as Executor
from std_msgs.msg import String
from hmi_keyboard_client import KeyboardCommands

class Harness:
    DS_PAUSED=0;DS_RUNNING=1;DS_ESTOP=2
    _on_paused_reset=Executor._on_paused_reset
    _make_stop_command=Executor._make_stop_command
    _update_safety_state=Executor._update_safety_state
    _try_clear_estop=Executor._try_clear_estop
    _update_control_mode=Executor._update_control_mode
    _select_active_cmd=Executor._select_active_cmd
    _on_websocket_cmd=Executor._on_websocket_cmd
    loop=Executor.loop
    def __init__(self):
        self.remote_req=False; self.abort_req=False;self.estop_reason='drive_cmd'
        self.estop_latched=True;self.drive_state=2;self.teleop_active=False
        self.paused_reset_guards={name:(True,time.monotonic()) for name in ('围栏','9010链路','外设及控制节点')}
        self.pub_paused_reset_result=Mock()
        self.cmd_teleop=self._make_stop_command();self.cmd_teleop.pre_speed_kmh=4.
        self.cmd_traj=None;self.cmd_bale=None;self.bale_active=False
        self.reset_estop_req=False
        self.enable_can=True;self.can_bus=object();self.send_can=Mock()
        self._log_state_if_needed=lambda:None
    def request(self):
        self._on_paused_reset(String(data=json.dumps({'request_id':'test'})))
        return json.loads(self.pub_paused_reset_result.publish.call_args.args[0].data)

class ResetTests(unittest.TestCase):
    def test_reset_is_atomic_zero_and_paused(self):
        node=Harness();result=node.request()
        self.assertTrue(result['ok']);self.assertFalse(node.estop_latched)
        self.assertEqual(node.drive_state,0);self.assertEqual(node.cmd_teleop.pre_speed_kmh,0)
        self.assertTrue(node.teleop_active)
        node.loop()
        out=node.send_can.call_args.kwargs
        self.assertFalse(out['drive']);self.assertFalse(out['estop'])
        self.assertEqual(out['speed_kmh'],0)
    def test_pick_dump_reach_can_with_drive_false(self):
        for field in ('pick','dump'):
            node=Harness();self.assertTrue(node.request()['ok'])
            cmd=node._make_stop_command();setattr(cmd,field,True)
            node._on_websocket_cmd(cmd);node.loop()
            out=node.send_can.call_args.kwargs
            self.assertTrue(out[field]);self.assertFalse(out['drive'])
            self.assertEqual((out['speed_kmh'],out['angle_deg']),(0,0))
    def test_abort_latch_is_not_overridden(self):
        node=Harness();node.abort_req=True
        self.assertFalse(node.request()['ok']);self.assertTrue(node.estop_latched)
        self.assertEqual(node.drive_state,2)
    def test_each_guard_is_fail_closed(self):
        for name in ('围栏','9010链路','外设及控制节点'):
            for value,stamp in [(False,time.monotonic()),(True,time.monotonic()-3),(False,None)]:
                node=Harness();node.paused_reset_guards[name]=(value,stamp)
                self.assertFalse(node.request()['ok']);self.assertTrue(node.estop_latched)
    def test_physical_remote_rejects(self):
        node=Harness();node.remote_req=True
        self.assertFalse(node.request()['ok'])
    def test_keyboard_r_uses_new_paused_protocol(self):
        keys=KeyboardCommands(work_only=True)
        self.assertEqual(keys.message('r')['type'],'reset_estop_paused')
        self.assertEqual(KeyboardCommands(work_only=False).message('r')['type'],'reset_estop')

if __name__=='__main__':unittest.main(verbosity=2)
