from pathlib import Path
import sys,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from obstacle_warning import summarize

class WarningTests(unittest.TestCase):
    def test_near_channel(self):
        result=summarize([.35,1,2,3],10,10.1)
        self.assertTrue(result['warning']);self.assertEqual(result['alarm_channels'],['front_left'])
    def test_clear_and_threshold_boundary(self):
        result=summarize([.5,1,2,3],10,10)
        self.assertEqual(result['state'],'warning')
        self.assertEqual(result['channel_states']['front_left'],'warning')
        self.assertEqual(result['channel_states']['rear_left'],'warning')
        self.assertEqual(result['channel_states']['rear_right'],'clear')
        self.assertEqual(summarize([2.01,3,4,5],10,10)['state'],'clear')
    def test_zero_is_unknown_not_obstacle_or_safe(self):
        result=summarize([0,0,0,0],10,10)
        self.assertIsNone(result['warning']);self.assertEqual(result['state'],'unknown')
    def test_partial_unknown_and_known_obstacle(self):
        result=summarize([0,.2,-1,float('nan')],10,10)
        self.assertTrue(result['warning']);self.assertEqual(len(result['unknown_channels']),3)
    def test_stale_discards_distance_and_alarm(self):
        result=summarize([.2,1,2,3],10,13)
        self.assertIsNone(result['warning']);self.assertTrue(all(v is None for v in result['distances_m'].values()))
    def test_channel_levels_and_priority(self):
        result=summarize([.49,.5,2.0,2.01],10,10)
        self.assertEqual(list(result['channel_states'].values()),['alarm','warning','warning','clear'])
        self.assertEqual(result['state'],'alarm')
    def test_unknown_does_not_hide_known_danger(self):
        self.assertEqual(summarize([0,.4,3,4],10,10)['state'],'alarm')
        self.assertEqual(summarize([0,1,3,4],10,10)['state'],'warning')
        self.assertEqual(summarize([0,3,3,4],10,10)['state'],'unknown')

    def test_never_received(self):
        self.assertEqual(summarize(None,None,10)['state'],'unknown')
if __name__=='__main__':unittest.main(verbosity=2)
