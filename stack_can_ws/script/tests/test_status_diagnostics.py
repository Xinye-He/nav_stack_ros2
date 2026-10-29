import sys,unittest
from pathlib import Path
from types import SimpleNamespace as O
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from status_diagnostics import DiagnosticEvents

def log(name,msg,level=30):return O(name=name,msg=msg,level=level,stamp=O(sec=1,nanosec=2))
class Tests(unittest.TestCase):
    def test_geofence_trigger_and_warning_are_distinct(self):
        d=DiagnosticEvents();d.record(log('geofence_monitor','Geofence: OUTSIDE! test',40));d.record(log('geofence_monitor','Geofence: publish /abort = True (ESTOP)'))
        self.assertEqual([e['evidence'] for e in d.events],['related_log','trigger_report'])
    def test_motor_network_executor(self):
        d=DiagnosticEvents()
        for m in [log('motor_protection_node','MOTOR PROTECTION ABORT: left stall',40),log('websocket_teleop_bridge','NETWORK FAULT: heartbeat timeout'),log('stack_can_executor','safety=ESTOP, estop_reason=abort',20)]:d.record(m)
        self.assertEqual(len(d.events),3);self.assertEqual(d.events[-1]['evidence'],'related_log')
        self.assertEqual(d.events[0]['source'],'motor_protection_node')
    def test_bounded_dedup_and_no_invented_cause(self):
        d=DiagnosticEvents(2)
        for _ in range(10):d.record(log('stack_can_executor','safety=ESTOP, estop_reason=abort',20))
        self.assertEqual(len(d.events),1)
        d.record(log('unrelated','abort',40));self.assertEqual(len(d.events),1)
        for n in range(4):d.record(log('ultrasonic',str(n),40))
        self.assertEqual(len(d.events),2);self.assertNotIn('root_cause',d.snapshot())
    def test_boot_and_timestamps(self):
        d=DiagnosticEvents();d.set_boot('a');d.record(log('websocket_teleop_bridge','NETWORK FAULT: disconnected'))
        self.assertEqual(d.events[0]['ros_stamp']['sec'],1)
        self.assertEqual(d.snapshot()['coverage'],'since_status_service_start')
        d.set_boot('b');self.assertFalse(d.events)
if __name__=='__main__':unittest.main(verbosity=2)
