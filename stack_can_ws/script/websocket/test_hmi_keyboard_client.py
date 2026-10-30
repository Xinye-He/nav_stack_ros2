import asyncio
from contextlib import contextmanager, redirect_stdout
import io
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import hmi_keyboard_client as hmi

class MappingTests(unittest.TestCase):
    def test_restart_confirmation_and_results(self):
        for work_only in (True, False):
            keys = hmi.KeyboardCommands(work_only=work_only)
            with patch.object(hmi.time, 'monotonic', side_effect=[0, 4, 5]), redirect_stdout(io.StringIO()):
                self.assertIsNone(keys.message('!'))
                self.assertIsNone(keys.message('!'))  # Expired confirmation rearms.
                request = keys.message('!')
                self.assertEqual(request['action'], 'restart')
                self.assertTrue(request['confirm_stopped'])
                self.assertIsNone(keys.message('!'))
                self.assertIsNone(keys.message('g'))
                self.assertEqual(keys.message('9')['action'], 'status')
                self.assertEqual(keys.message('x')['state'], 2)
                result = dict(type='device_result', action='restart', request_id=request['request_id'], ok=False)
                keys.observe_result(dict(result, code='RESTART_RESULT_UNKNOWN'))
                self.assertTrue(keys.restart_pending)
                keys.observe_result(dict(result, code='DEVICE_REQUEST_REJECTED'))
                self.assertFalse(keys.restart_pending)

    def test_other_key_cancels_restart_confirmation(self):
        keys = hmi.KeyboardCommands()
        with redirect_stdout(io.StringIO()):
            keys.message('!')
            self.assertEqual(keys.message('9')['action'], 'status')
            self.assertIsNone(keys.message('!'))
            self.assertFalse(keys.restart_pending)

    def test_motion_and_stop(self):
        keys=hmi.KeyboardCommands()
        self.assertEqual(keys.message('m'), {'type':'active','value':True})
        self.assertEqual(keys.message('g'), {'type':'drive','state':1})
        self.assertEqual(keys.message('w')['pre_speed_kmh'],4.0)
        self.assertEqual(keys.message('q')['angle_deg'],-20.0)
        stopped=keys.message(' ')
        self.assertEqual((stopped['pre_speed_kmh'],stopped['angle_deg']),(0,0))
        self.assertEqual(keys.message('u'),{'type':'active','value':False})

    def test_work_only_blocks_movement_and_all_other_jobs(self):
        keys=hmi.KeyboardCommands(work_only=True)
        for key in 'w4s18adqegzumtlhvbp':
            with redirect_stdout(io.StringIO()): self.assertIsNone(keys.message(key))
        dump=keys.message('j')
        self.assertTrue(dump['dump'])
        pick=keys.message('k')
        self.assertTrue(pick['pick']);self.assertFalse(pick['dump'])
        self.assertEqual((pick['pre_speed_kmh'],pick['angle_deg']),(0,0))
        self.assertFalse(pick['unload']);self.assertFalse(pick['pick_action'])
        cleared=keys.message(' ')
        self.assertFalse(cleared['pick']);self.assertFalse(cleared['dump'])

    def test_jobs_reset_and_camera(self):
        keys=hmi.KeyboardCommands()
        keys.message('j')
        self.assertFalse(keys.message('k')['dump'])
        self.assertTrue(keys.message('h')['pick_action'])
        with redirect_stdout(io.StringIO()): self.assertIsNone(keys.message('j'))
        keys.message('r')
        self.assertFalse(keys.command['pick_action'])
        self.assertEqual(keys.message('v')['action'],'start')
        self.assertEqual(keys.message('b')['action'],'stop')
        self.assertEqual(keys.message('p')['action'],'status')

class LoopbackTests(unittest.IsolatedAsyncioTestCase):
    async def test_keyboard_heartbeat_camera_and_status(self):
        try: from websockets.legacy.server import serve
        except ImportError: from websockets import serve
        messages=[]
        async def control(ws,path):
            async for raw in ws:
                msg=json.loads(raw);messages.append(msg)
                if msg.get('type')=='camera':
                    await ws.send(json.dumps({'type':'camera_result','request_id':msg['request_id'],'ok':True}))
        async def status(ws,path):
            await ws.send(json.dumps({'type':'device_status','device_state':'ready','vehicle_state':None,'peripherals':{}}))
            await ws.wait_closed()
        async with serve(control,'127.0.0.1',0) as ctrl, serve(status,'127.0.0.1',0) as stat:
            queue=asyncio.Queue()
            @contextmanager
            def keyboard(): yield queue
            args=SimpleNamespace(host='127.0.0.1',control_port=ctrl.sockets[0].getsockname()[1],
                status_port=stat.sockets[0].getsockname()[1],csv_port=0,
                status_only=False,dry_run=False,keys=None,verbose=False,upload_fence=None,upload_points=None,work_only=False)
            async def keys():
                await asyncio.sleep(.1)
                for key in 'mgwacp':await queue.put(key)
                await asyncio.sleep(.7)
                await queue.put('\x1b')
            out=io.StringIO()
            with patch.object(hmi,'terminal_queue',keyboard),redirect_stdout(out):
                await asyncio.gather(hmi.run(args),keys())
            self.assertGreaterEqual(sum(m.get('type')=='heartbeat' for m in messages),2)
            self.assertTrue(any(m.get('pre_speed_kmh')==4 for m in messages))
            self.assertTrue(any(m.get('type')=='camera' for m in messages))
            self.assertEqual(messages[-1]['pre_speed_kmh'],0)
            self.assertIn('camera_result',out.getvalue())
            self.assertIn('就绪',out.getvalue())

    async def test_work_only_starts_paused_and_never_sends_motion(self):
        try: from websockets.legacy.server import serve
        except ImportError: from websockets import serve
        messages=[]
        async def control(ws,path):
            async for raw in ws: messages.append(json.loads(raw))
        async def status(ws,path):
            await ws.send(json.dumps({'device_state':'ready','vehicle_state':None,'peripherals':{}}))
            await ws.wait_closed()
        async with serve(control,'127.0.0.1',0) as ctrl, serve(status,'127.0.0.1',0) as stat:
            queue=asyncio.Queue()
            @contextmanager
            def keyboard(): yield queue
            args=SimpleNamespace(host='127.0.0.1',control_port=ctrl.sockets[0].getsockname()[1],
                status_port=stat.sockets[0].getsockname()[1],csv_port=0,status_only=False,
                dry_run=False,keys=None,verbose=False,upload_fence=None,upload_points=None,work_only=True)
            async def keys():
                await asyncio.sleep(.1)
                for key in 'jkwgsad': await queue.put(key)
                await asyncio.sleep(.1)
                await queue.put('\x1b')
            with patch.object(hmi,'terminal_queue',keyboard),redirect_stdout(io.StringIO()):
                await asyncio.gather(hmi.run(args),keys())
        self.assertEqual(messages[0],{'type':'drive','state':0})
        self.assertEqual(messages[1]['pre_speed_kmh'],0)
        self.assertEqual(messages[2],{'type':'active','value':True})
        for msg in messages:
            if msg['type']=='drive': self.assertEqual(msg['state'],0)
            self.assertNotEqual(msg['type'],'reset_estop')
            if msg['type']=='cmd':
                self.assertEqual((msg['pre_speed_kmh'],msg['angle_deg']),(0,0))
                self.assertFalse(msg['unload']);self.assertFalse(msg['pick_action'])
        self.assertTrue(any(m.get('dump') for m in messages))
        self.assertTrue(any(m.get('pick') for m in messages))

    async def test_csv_upload_does_not_open_control(self):
        import tempfile
        try: from websockets.legacy.server import serve
        except ImportError: from websockets import serve
        captured=[]
        async def csv(ws,path):
            captured.append((path,await ws.recv()))
            await ws.send('saved:fence.csv, rows=2, valid=True')
        async with serve(csv,'127.0.0.1',0) as server:
            with tempfile.TemporaryDirectory() as tmp:
                path=Path(tmp)/'fence.csv';path.write_text('idx,lat,lon\n0,29,119\n')
                args=SimpleNamespace(host='127.0.0.1',csv_port=server.sockets[0].getsockname()[1],
                    dry_run=False,upload_fence=str(path),upload_points=None)
                with redirect_stdout(io.StringIO()):await hmi.run(args)
        self.assertEqual(captured,[('/fence','idx,lat,lon\n0,29,119\n')])

if __name__=='__main__':unittest.main(verbosity=2)
