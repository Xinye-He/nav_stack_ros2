#!/usr/bin/env python3
"""Independent host restart coordinator. Only one fixed systemd unit is allowed."""
import argparse
import asyncio
from collections import OrderedDict
import json
import os
from pathlib import Path
import tempfile
import time
import uuid

BASE = Path(__file__).resolve().parent
UNIT = 'nav-stack-ros2.service'


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.restart-', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, ensure_ascii=False, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, str(path))
    finally:
        if os.path.exists(name):
            os.unlink(name)


async def restart_navigation():
    proc = await asyncio.create_subprocess_exec('/bin/systemctl', 'restart', UNIT,
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
    try:
        out, err = await asyncio.wait_for(proc.communicate(), 60)
    except asyncio.TimeoutError:
        proc.kill()
        await proc.wait()
        raise TimeoutError('systemctl 超时，重启结果未知，禁止自动重试')
    if proc.returncode:
        raise RuntimeError(err.decode(errors='replace')[:800] or 'systemctl restart 失败')


class Controller:
    def __init__(self, runtime, restart=None, delay=2.0):
        self.runtime = Path(runtime)
        self.runtime.mkdir(parents=True, exist_ok=True)
        self.marker = self.runtime / 'start_paused.json'
        self.state_file = self.runtime / 'jobs.json'
        self.restart = restart or restart_navigation
        self.delay = delay
        self.jobs = OrderedDict()
        self.tasks = set()
        try:
            values = json.loads(self.state_file.read_text())
            for job in values[-100:]:
                if job['state'] == 'prepared':
                    job.update(state='expired', message='服务重启前未提交，不执行重启')
                elif job['state'] in ('scheduled', 'restarting'):
                    job.update(state='unknown', message='管理服务曾重启，前次操作结果未知；不自动重试')
                self.jobs[job['request_id']] = job
        except (OSError, ValueError, KeyError, TypeError):
            pass

    def persist(self):
        while len(self.jobs) > 100:
            self.jobs.popitem(last=False)
        atomic_json(self.state_file, list(self.jobs.values()))

    def refresh(self):
        for job in self.jobs.values():
            if job['state'] == 'prepared' and time.monotonic() > job['expires_at']:
                job.update(state='expired', message='准备请求过期，未执行重启')

    def busy(self):
        self.refresh()
        return any(j['state'] in ('prepared', 'scheduled', 'restarting', 'unknown') for j in self.jobs.values())

    def public_job(self, job):
        return {k:v for k,v in job.items() if k not in ('token', 'expires_at')}

    def proof_valid(self, request):
        proof = request.get('safety_proof')
        return (request.get('confirm_stopped') is True and isinstance(proof, dict)
            and proof.get('request_id') == request.get('request_id')
            and proof.get('mode') in ('executor_paused', 'startup_failed_no_executor')
            and isinstance(proof.get('checked_at'), (int, float))
            and 0 <= time.monotonic() - proof['checked_at'] <= 2.0)

    async def execute(self, request):
        rid, action = request.get('request_id'), request.get('action')
        base = dict(type='device_result', request_id=rid, action='status' if action == 'status' else 'restart', service=UNIT)
        if not isinstance(rid, str) or not 1 <= len(rid) <= 128 or action not in ('status','prepare','commit','cancel'):
            return dict(base, ok=False, code='INVALID_REQUEST', message='无效的设备管理请求')
        self.refresh()
        if action == 'status':
            job = self.jobs.get(request.get('job_id')) if request.get('job_id') else next(reversed(self.jobs.values()), None) if self.jobs else None
            return dict(base, ok=True, code='OK', startup_pause_armed=self.marker.exists(), job=self.public_job(job) if job else None)
        job = self.jobs.get(rid)
        if action == 'prepare':
            if job:
                return dict(base, ok=True, code='RESTART_ALREADY_REQUESTED', token=job['token'], job=self.public_job(job))
            if self.busy():
                return dict(base, ok=False, code='DEVICE_BUSY', message='存在未完成或结果未知的重启请求，请先检查状态')
            if not self.proof_valid(request):
                return dict(base, ok=False, code='DEVICE_SAFETY_REJECTED', message='缺少停止确认或新鲜的执行器暂停维护证明')
            job = dict(request_id=rid, token=uuid.uuid4().hex, state='prepared',
                expires_at=time.monotonic()+15, prepared_at=time.time(), boot_id_before=request.get('boot_id_before'))
            self.jobs[rid] = job
            self.persist()
            return dict(base, ok=True, code='RESTART_PREPARED', token=job['token'], job=self.public_job(job))
        if not job or request.get('token') != job['token']:
            return dict(base, ok=False, code='INVALID_REQUEST', message='重启令牌不匹配')
        if action == 'cancel':
            if job['state'] != 'prepared':
                return dict(base, ok=False, code='RESTART_NOT_CANCELABLE', job=self.public_job(job))
            job.update(state='cancelled', message='准备阶段已取消，未执行重启');self.persist()
            return dict(base, ok=True, code='RESTART_CANCELLED', job=self.public_job(job))
        if job['state'] != 'prepared':
            return dict(base, ok=job['state'] in ('scheduled','restarting','completed'), code='RESTART_ALREADY_REQUESTED', job=self.public_job(job))
        if not self.proof_valid(request):
            return dict(base, ok=False, code='DEVICE_SAFETY_REJECTED', message='提交前暂停维护证明已过期')
        # Persistent marker is written before scheduling and is never removed by
        # startup. A failed launch or later reboot must also remain paused.
        atomic_json(self.marker, dict(request_id=rid, armed_at=time.time(), startup_mode='paused'))
        job.update(state='scheduled', scheduled_at=time.time(), restart_after_sec=self.delay, startup_mode='paused')
        self.persist()
        task = asyncio.create_task(self.run_job(job));self.tasks.add(task);task.add_done_callback(self.tasks.discard)
        return dict(base, ok=True, code='RESTART_SCHEDULED', message='已安排导航重启，启动后保持暂停', job=self.public_job(job))

    async def run_job(self, job):
        await asyncio.sleep(self.delay)
        job.update(state='restarting', started_at=time.time());self.persist()
        try:
            await self.restart()
            job.update(state='completed', message='systemctl 已成功返回，仍需通过新boot_id和9011启动状态确认导航就绪')
        except TimeoutError as exc:
            job.update(state='unknown', message=str(exc))
        except (OSError, RuntimeError) as exc:
            job.update(state='failed', message=str(exc))
        job['finished_at'] = time.time();self.persist()


async def run(path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True);os.chmod(path.parent,0o770)
    if path.exists():
        try:
            reader, writer = await asyncio.open_unix_connection(str(path))
        except (ConnectionRefusedError, FileNotFoundError):
            path.unlink(missing_ok=True)
        else:
            writer.close();await writer.wait_closed();raise RuntimeError('设备管理服务已运行')
    controller = Controller(path.parent)
    async def handler(reader, writer):
        try:
            value = json.loads(await asyncio.wait_for(reader.readline(),3))
            if not isinstance(value,dict):raise ValueError('请求须为JSON对象')
            response = await controller.execute(value)
            writer.write((json.dumps(response,ensure_ascii=False,allow_nan=False)+'\n').encode());await writer.drain()
        except (ValueError,OSError,asyncio.TimeoutError) as exc:
            try:
                writer.write((json.dumps(dict(type='device_result',ok=False,code='INVALID_REQUEST',message=str(exc)))+'\n').encode());await writer.drain()
            except OSError:pass
        finally:
            writer.close()
            try:await writer.wait_closed()
            except OSError:pass
    server = await asyncio.start_unix_server(handler,path=str(path),limit=16384)
    os.chmod(path,0o660)
    try:
        async with server:await server.serve_forever()
    finally:path.unlink(missing_ok=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--socket',default=str(BASE/'.runtime/control.sock'))
    try:asyncio.run(run(parser.parse_args().socket))
    except KeyboardInterrupt:pass
