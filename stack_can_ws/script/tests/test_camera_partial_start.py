import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import AsyncMock, patch

BASE=Path(__file__).resolve().parents[1]/'video_usb'
spec=importlib.util.spec_from_file_location('camera_partial',BASE/'camera_control_service.py')
host=importlib.util.module_from_spec(spec);spec.loader.exec_module(host)
def snapshot(ready=False, usb=False, usb_ready=False):
    return {'camera_state':'running' if ready and usb_ready else ('partial' if ready else 'stopped'),
      'cameras':[{'name':'cam0','device':'/dev/null','device_present':True,'stream_ready':ready},
                 {'name':'cam1','device':'/dev/not-present-test','device_present':usb,'stream_ready':usb_ready}]}

class PartialTests(unittest.IsolatedAsyncioTestCase):
    async def test_csi_starts_without_usb(self):
        ctl=host.CameraController('jetson');ctl.status=AsyncMock(side_effect=[snapshot(),snapshot(True)])
        with patch.object(host,'command',AsyncMock(return_value=(0,'skipped cam1'))) as cmd:
            result=await ctl.execute({'action':'start','request_id':'1'})
            self.assertTrue(result['ok']);self.assertEqual(result['code'],'PARTIAL_READY')
            self.assertEqual(result['camera_state'],'partial');cmd.assert_awaited_once()
    async def test_partial_start_is_idempotent(self):
        ctl=host.CameraController('jetson');ctl.status=AsyncMock(return_value=snapshot(True))
        with patch.object(host,'command',AsyncMock()) as cmd:
            result=await ctl.execute({'action':'start','request_id':'2'})
            self.assertEqual(result['code'],'PARTIAL_READY');cmd.assert_not_awaited()
    async def test_both_cameras_ready(self):
        ctl=host.CameraController('jetson');ctl.status=AsyncMock(side_effect=[snapshot(usb=True),snapshot(True,True,True)])
        with patch.object(host,'command',AsyncMock(return_value=(0,'done'))):
            result=await ctl.execute({'action':'start','request_id':'3'})
            self.assertTrue(result['ok']);self.assertEqual(result['code'],'OK')
    async def test_no_devices_and_failure_are_not_success(self):
        ctl=host.CameraController('jetson');empty=snapshot();empty['cameras'][0]['device_present']=False
        ctl.status=AsyncMock(return_value=empty)
        with patch.object(host,'command',AsyncMock()) as cmd:
            self.assertEqual((await ctl.execute({'action':'start','request_id':'4'}))['code'],'CAMERA_DEVICE_MISSING')
            cmd.assert_not_awaited()
        ctl.status=AsyncMock(return_value=snapshot())
        with patch.object(host,'command',AsyncMock(return_value=(1,'failed'))):
            self.assertFalse((await ctl.execute({'action':'start','request_id':'5'}))['ok'])
    async def test_stop_partial(self):
        ctl=host.CameraController('jetson');ctl.status=AsyncMock(side_effect=[snapshot(True),snapshot()])
        with patch.object(host,'command',AsyncMock(return_value=(0,'stopped'))):
            self.assertTrue((await ctl.execute({'action':'stop','request_id':'6'}))['ok'])
    async def test_shell_skips_missing_camera(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);bin_dir=root/'bin';bin_dir.mkdir()
            for name in ('start_monitor_new.sh','stop_monitor.sh'):shutil.copy2(BASE/name,root/name)
            (root/'config.sh').write_text('CAM_NAMES=(cam0 cam1)\nCAM_DEVS=(/dev/null /dev/no-camera-test)\nCAM_TYPES=(csi usb)\nCSI_SENSOR_ID=0\nCAM_WIDTH=1920\nCAM_HEIGHT=1080\nCAM_FPS=30\nCAM_BITRATE_KBPS=2000\nRTSP_SERVER=127.0.0.1\nRTSP_PORT=8554\n')
            for name in ('docker','gst-launch-1.0','sleep'):
                tool=bin_dir/name;tool.write_text('#!/bin/bash\nexit 0\n');tool.chmod(0o755)
            run=subprocess.run(['bash',str(root/'start_monitor_new.sh')],env=dict(os.environ,PATH=str(bin_dir)+':'+os.environ['PATH']),capture_output=True,text=True,timeout=5)
            self.assertEqual(run.returncode,0,run.stderr)
            self.assertIn('跳过未识别相机：cam1',run.stdout)
            self.assertTrue((root/'cam0.pid').exists());self.assertFalse((root/'cam1.pid').exists())
if __name__=='__main__':unittest.main(verbosity=2)
