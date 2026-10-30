"""Two-phase host restart client. Reuses the executor's maintenance interlock."""
import asyncio
from contextlib import suppress
import json
import os
import time
from pathlib import Path
import uuid


async def host_request(request):
    default = Path(os.environ.get('ROS_WS', '/root/stack_can_ws')) / 'script/device_control/.runtime/control.sock'
    path = os.environ.get('DEVICE_CONTROL_SOCKET', str(default))
    writer = None
    try:
        reader, writer = await asyncio.wait_for(asyncio.open_unix_connection(path), 2)
        writer.write((json.dumps(request, allow_nan=False)+'\n').encode());await writer.drain()
        value = json.loads(await asyncio.wait_for(reader.readline(), 5))
        if not isinstance(value, dict) or value.get('request_id') != request['request_id']:
            raise ValueError('设备管理回执不匹配')
        return value
    finally:
        if writer:
            writer.close()
            with suppress(OSError):await writer.wait_closed()


class DeviceControlClient:
    def __init__(self, node, can_client):
        self.node, self.can = node, can_client
        self.pending = False
        self.tasks = set()

    async def send(self, ws, result):
        await ws.send(json.dumps(result, ensure_ascii=False, allow_nan=False))

    async def submit(self, ws, data):
        rid, action = data.get('request_id', str(uuid.uuid4())), data.get('action')
        base = dict(type='device_result', request_id=rid, action=action)
        if action not in ('status','restart') or not isinstance(rid,str) or not 1 <= len(rid) <= 128:
            await self.send(ws,dict(base,ok=False,code='INVALID_REQUEST',message='设备操作须为status/restart，request_id须为1~128字符'));return
        if action == 'restart' and data.get('confirm_stopped') is not True:
            await self.send(ws,dict(base,ok=False,code='DEVICE_STOP_CONFIRMATION_REQUIRED',message='请先确认车辆停稳，发送confirm_stopped=true'));return
        if action == 'restart' and (self.pending or self.can.pending):
            await self.send(ws,dict(base,ok=False,code='DEVICE_BUSY',message='CAN/导航维护正在进行，请先查询状态'));return
        if action == 'restart':
            self.pending=True;self.can.pending=True
        task=asyncio.create_task(self.execute(ws,data,base));self.tasks.add(task);task.add_done_callback(self.tasks.discard)

    async def execute(self, ws, data, base):
        rid, action = base['request_id'], base['action']
        token=None;commit_attempted=False;keep_hold=False
        try:
            if action=='status':
                result=await host_request(dict(action='status',request_id=rid))
                await self.send(ws,result);return
            proof=await self.can.safe_proof(rid)
            try:
                lifecycle=json.loads(Path(os.environ.get('NAV_STATUS_FILE','/tmp/nav_device_status.json')).read_text())
            except (OSError,ValueError):lifecycle={}
            prepared=await host_request(dict(action='prepare',request_id=rid,confirm_stopped=True,safety_proof=proof,boot_id_before=lifecycle.get('boot_id')))
            if not prepared.get('ok'):
                await self.send(ws,prepared);return
            token=prepared['token'];state=prepared['job']['state']
            if state!='prepared':
                await self.send(ws,dict(base,ok=state in ('scheduled','restarting','completed'),code='RESTART_ALREADY_REQUESTED',job=prepared['job']))
                if state in ('scheduled','restarting','unknown'):
                    keep_hold=True
                    keep_hold=await self.watch(ws,rid,base)
                return
            # A failed websocket send never commits the host operation.
            await self.send(ws,dict(base,ok=True,code='RESTART_ACCEPTED',
                message='已接受重启请求，随后9010/9011将断开；请重连并通过新boot_id检查启动状态',
                startup_mode='paused',boot_id_before=lifecycle.get('boot_id')))
            proof=await self.can.safe_proof(rid)
            commit_attempted=True;keep_hold=True
            committed=await host_request(dict(action='commit',request_id=rid,token=token,confirm_stopped=True,safety_proof=proof))
            if not committed.get('ok'):
                keep_hold=committed.get('job',{}).get('state') in ('scheduled','restarting','unknown')
                await self.send(ws,committed)
                if keep_hold:keep_hold=await self.watch(ws,rid,base)
                return
            with suppress(Exception):await self.send(ws,committed)
            keep_hold=await self.watch(ws,rid,base)
        except Exception as exc:
            # Before commit no restart is possible. After a lost commit reply,
            # keep paused until the host can positively report cancellation/failure.
            keep_hold=commit_attempted
            with suppress(Exception):
                await self.send(ws,dict(base,ok=False,code='RESTART_RESULT_UNKNOWN' if commit_attempted else 'DEVICE_REQUEST_REJECTED',
                    message=str(exc).replace('恢复 CAN','重启导航'),maintenance_held=keep_hold))
            if commit_attempted:
                keep_hold=await self.watch(ws,rid,base)
        finally:
            if action=='restart' and not keep_hold:
                if token:
                    with suppress(Exception):await host_request(dict(action='cancel',request_id=rid,token=token))
                if self.can.maintenance_rid==rid:
                    self.can.zero_and_pause();self.can.maintenance(rid,False)
                self.can.pending=False;self.pending=False

    async def watch(self,ws,rid,base):
        # The usual successful path kills this process as part of the restart.
        # The host coordinator survives and owns completion tracking.
        deadline=time.monotonic()+90
        while time.monotonic()<deadline:
            await asyncio.sleep(1)
            try:
                result=await host_request(dict(action='status',request_id='watch-'+uuid.uuid4().hex,job_id=rid))
                job=result.get('job') or {};state=job.get('state')
                if state in ('failed','cancelled','expired'):
                    with suppress(Exception):await self.send(ws,dict(base,ok=False,code='RESTART_FAILED',message=job.get('message','导航重启未执行'),job=job))
                    return False
                if state in ('completed','unknown'):
                    with suppress(Exception):await self.send(ws,dict(base,ok=False,code='RESTART_VERIFY_REQUIRED',message='请通过9011新boot_id确认；当前调用进程仍存活，保持维护暂停',job=job))
                    return True
            except Exception:
                pass
        with suppress(Exception):await self.send(ws,dict(base,ok=False,code='RESTART_RESULT_UNKNOWN',message='未能确认重启结果，保持暂停，请人工检查宿主机服务'))
        return True
