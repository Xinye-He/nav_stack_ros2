"""Two-phase path reload/start protocol. No implicit RUN on reload."""
import asyncio
import csv
import hashlib
import io
import json
import math
import time
import uuid
from pathlib import Path as FilePath
from std_msgs.msg import String, UInt8
from main.paused_reset import rejection


def load_snapshot(filename):
    content = FilePath(filename).read_bytes()
    rows=[]
    for lineno,row in enumerate(csv.reader(io.StringIO(content.decode('utf-8-sig'))),1):
        if not row or not any(v.strip() for v in row) or row[0].lstrip().startswith('#'):
            continue
        if not rows and row[0].strip().lower() in ('idx','index','id'):
            continue
        try:
            idx=int(row[0]);lat=float(row[1]);lon=float(row[2]);heading=float(row[3])
            kind=int(row[4]) if len(row)>4 else 0
            if not all(math.isfinite(v) for v in (lat,lon,heading)) or not -90<=lat<=90 or not -180<=lon<=180:
                raise ValueError('坐标或航向无效')
            rows.append((idx,lat,lon,heading,kind))
        except (ValueError,IndexError) as exc:
            raise ValueError('路径第{}行无效：{}'.format(lineno,exc))
    if not rows:raise ValueError('路径没有有效路点')
    if len({r[0] for r in rows})!=len(rows):raise ValueError('路径索引重复')
    return sorted(rows),hashlib.sha256(content).hexdigest()


def publish(pub,data):
    pub.publish(String(data=json.dumps(data,ensure_ascii=False)))


def request_data(msg):
    data=json.loads(msg.data)
    if not isinstance(data,dict) or data.get('action') not in ('reload','start','remote','select'):
        raise ValueError('无效路径操作')
    rid=data.get('request_id')
    if not isinstance(rid,str) or not 1<=len(rid)<=128:raise ValueError('无效request_id')
    return data


class FollowerPathControl:
    def __init__(self,node,geo_class,heading_converter):
        self.node=node;self.geo_class=geo_class;self.heading_converter=heading_converter
        self.version=None
        # The follower has already loaded its geometry before constructing us.
        if getattr(node, 'waypoints_xy', None):
            try: _, self.version = load_snapshot(node.path_csv)
            except (OSError, ValueError, UnicodeError, csv.Error): pass
        self.position_received=None;self.heading_received=None
        self.pub=node.create_publisher(String,'/trajectory/internal_result',10)
        node.create_subscription(String,'/trajectory/internal_request',self.handle,10)

    def handle(self,msg):
        try:req=request_data(msg)
        except (ValueError,TypeError):return
        node=self.node;action=req['action']
        result={'request_id':req['request_id'],'action':action,'ok':False}
        try:
            if action in ('remote', 'select'):
                node.drive_state=node.DS_ESTOP if node.abort_req else node.DS_PAUSED
                node.publish_traj_cmd(0.0,0.0,0.0)
                result.update(ok=True,code='OK',message='循迹已暂停',path_version=self.version)
            elif action=='reload':
                node.drive_state=node.DS_ESTOP if node.abort_req else node.DS_PAUSED
                node.publish_traj_cmd(0.0,0.0,0.0)
                rows,version=load_snapshot(node.path_csv)
                # Build everything before replacing the old geometry.
                geo=self.geo_class(rows[0][1],rows[0][2],0.0)
                points=[]
                from nav_msgs.msg import Path
                from geometry_msgs.msg import PoseStamped
                path=Path();path.header.frame_id=node.map_frame
                for _,lat,lon,heading,kind in rows:
                    x,y,_=geo.lla_to_enu(lat,lon,0.0)
                    points.append((x,y,self.heading_converter(heading),kind))
                    pose=PoseStamped();pose.header.frame_id=node.map_frame
                    pose.pose.position.x=float(x);pose.pose.position.y=float(y);pose.pose.orientation.w=1.0
                    path.poses.append(pose)
                node.geo=geo;node.waypoints_xy=points;node.global_path=path
                node.seg_idx=0;node.aligned=False;node.waiting_for_task=False;node.task_done_latch=False
                node.unload_mode=False;node.unload_end_time=0.0;node.unload_reset_end_time=0.0
                node.unload_finished_once=False;node.unload_target_idx=-1;node.completed_unload_indices.clear()
                node.spin_state='IDLE';node.angle_sign=0;node.spin_pre_stop_until=0.0
                node.cur_x=None;node.cur_y=None;node.cur_yaw=None;node.dr_x=None;node.dr_y=None
                node.yaw_offset=math.radians(float(node.get_parameter('yaw_offset_deg').value))
                node.last_time=node.get_clock().now();node.last_pre_kmh_sent=0.0
                node._pub_waiting(False);node._pub_global_path();node.publish_traj_cmd(0.0,0.0,0.0)
                self.position_received=None;self.heading_received=None
                self.version=version
                result.update(ok=True,code='OK',message='路径已加载，保持暂停',path_version=version,
                              waypoint_count=len(points),path_file=node.path_csv)
            else:
                if node.abort_req or node.drive_state==node.DS_ESTOP:
                    raise ValueError('循迹节点存在中止锁存，请先处理保护条件')
                if not self.version or (req.get('path_version') is not None and req['path_version']!=self.version):
                    raise ValueError('路径版本不匹配，请先重新加载')
                _,disk_version=load_snapshot(node.path_csv)
                if disk_version!=self.version:raise ValueError('磁盘路径已更新，请重新加载')
                if (any(v is None or not math.isfinite(v) for v in (node.cur_x,node.cur_y,node.cur_yaw))
                        or self.position_received is None or self.heading_received is None
                        or time.monotonic()-self.position_received>2 or time.monotonic()-self.heading_received>2):
                    raise ValueError('新路径坐标系的定位或航向尚未更新或已过期')
                # Prepare only. The executor performs final checks and issues RUN.
                result.update(ok=True,code='OK',message='循迹节点准备完成',path_version=self.version,
                              waypoint_count=len(node.waypoints_xy))
        except (OSError,ValueError,UnicodeError,csv.Error) as exc:
            if action=='reload':self.version=None
            result.update(code='PATH_RELOAD_FAILED' if action=='reload' else 'TRAJECTORY_START_REJECTED',message=str(exc))
        publish(self.pub,result)


def geofence_rejection(guard, detail, received, now):
    ok, stamp = guard
    if stamp is None:
        return '尚未收到围栏判断消息 /geofence_ok，禁止启动循迹'
    if now - stamp > 2.0:
        return '围栏判断消息已超时：距上次接收 {:.1f} 秒，要求不超过 2.0 秒'.format(now - stamp)
    if ok:
        return ''
    if detail is None or received is None:
        return '围栏不允许启动（/geofence_ok=false），尚未收到详细状态 /geofence/status'
    if now - received > 2.0:
        return '围栏不允许启动，详细状态已超时：距上次接收 {:.1f} 秒'.format(now - received)
    code = detail.get('code')
    if code == 'NEAR_BOUNDARY':
        distance, margin = detail.get('distance_to_boundary_m'), detail.get('required_margin_m')
        if (isinstance(distance, (int, float)) and isinstance(margin, (int, float))
                and math.isfinite(distance) and math.isfinite(margin)):
            return '距围栏边界 {:.2f} m，要求至少 {:.2f} m，禁止启动循迹'.format(distance, margin)
        return '车辆靠近围栏边界，未满足安全距离，禁止启动循迹'
    if code == 'OUTSIDE':
        return '车辆定位在电子围栏外，禁止启动循迹'
    if code == 'POSITION_UNKNOWN':
        return '围栏位置未知：尚未收到可用 GPS 位置（{}），禁止启动循迹'.format(detail.get('gps_topic', '/fix_center'))
    return '围栏不允许启动：详细状态与 /geofence_ok 不一致，请等待状态更新'


class ExecutorPathControl:
    def __init__(self,node):
        self.node=node;self.pending=None;self.loaded_version=None;self.traj_received=None
        self.geofence_detail=None;self.geofence_received=None
        node.create_subscription(String,'/geofence/status',self.on_geofence_detail,10)
        self.pub=node.create_publisher(String,'/vehicle/trajectory_result',10)
        self.internal=node.create_publisher(String,'/trajectory/internal_request',10)
        self.drive=node.create_publisher(UInt8,'/drive_cmd',10)
        node.create_subscription(String,'/trajectory/control',self.handle,10)
        node.create_subscription(String,'/trajectory/internal_result',self.on_result,10)
        node.create_timer(0.1,self.check_timeout)

    def on_geofence_detail(self,msg):
        try:
            detail=json.loads(msg.data)
            if isinstance(detail,dict) and detail.get('type')=='geofence_status':
                self.geofence_detail=detail;self.geofence_received=time.monotonic()
        except (ValueError,TypeError):
            pass

    def pause(self):
        node=self.node
        node.drive_state=node.DS_ESTOP if node.estop_latched else node.DS_PAUSED
        node.reset_estop_req=False
        node.cmd_teleop=node._make_stop_command();node.teleop_active=True
        node.teleop_source='websocket_9010'

    def start_rejection(self,req):
        node=self.node
        if node.estop_latched or node.drive_state==node.DS_ESTOP or node.abort_req:
            return '急停或中止未解除，不允许启动循迹'
        reason=rejection({'remote_req':node.remote_req,'abort_req':node.abort_req,
            'estop_reason':node.estop_reason,
            'guards':[(name,)+node.paused_reset_guards.get(name,(False,None))
                      for name in ('围栏','9010链路','外设及控制节点')]},time.monotonic())
        if reason:
            if reason.startswith('围栏'):
                return geofence_rejection(node.paused_reset_guards.get('围栏',(False,None)),
                    self.geofence_detail,self.geofence_received,time.monotonic())
            return reason
        if req.get('path_version') is not None and req['path_version']!=self.loaded_version:
            return '请先成功重载路径，并提供对应path_version'
        if (self.traj_received is None or time.monotonic()-self.traj_received>2
                or node.cmd_traj is None or not node.cmd_traj.valid or not node._is_zero_cmd(node.cmd_traj)):
            return '循迹停止指令尚未就绪或已过期'
        return ''

    def reply(self,req,ok,code,message,**extra):
        publish(self.pub,dict(type='trajectory_result',request_id=req['request_id'],
            action=req['action'],ok=ok,code=code,message=message,
            drive_state=self.node.drive_state,**extra))

    def handle(self,msg):
        try:req=request_data(msg)
        except (ValueError,TypeError):return
        if getattr(self.node,'can_maintenance',None) is not None and self.node.can_maintenance.active:
            self.reply(req,False,'CAN_BUSY','CAN 恢复维护中，禁止操作路径');return
        if self.pending:
            self.reply(req,False,'TRAJECTORY_BUSY','另一条路径操作正在执行');return
        if req['action'] in ('remote','select') and (self.node.estop_latched or self.node.abort_req):
            self.reply(req,False,'TRAJECTORY_START_REJECTED','急停或中止未解除');return
        if req['action']=='start':
            reason=self.start_rejection(req)
            if reason:
                self.reply(req,False,'TRAJECTORY_START_REJECTED',reason);return
        self.pause()
        if req['action']=='reload':
            self.loaded_version=None;self.traj_received=None;self.node.cmd_traj=None
        self.pending=(req,time.monotonic())
        publish(self.internal,req)

    def on_result(self,msg):
        try:result=json.loads(msg.data)
        except (ValueError,TypeError):return
        if not isinstance(result,dict) or not self.pending:return
        req,_=self.pending
        if result.get('request_id')!=req['request_id'] or result.get('action')!=req['action']:return
        if getattr(self.node,'can_maintenance',None) is not None and self.node.can_maintenance.active:
            self.pending=None
            self.reply(req,False,'CAN_BUSY','CAN 恢复维护中，路径操作已取消');return
        if not result.get('ok'):
            self.pending=None
            self.reply(req,False,result.get('code','PATH_CONTROL_FAILED'),result.get('message','路径操作失败'));return
        if req['action']=='remote':
            self.pending=None
            node=self.node
            self.pause()
            if not node.estop_latched and not node.abort_req:
                node.drive_state=node.DS_RUNNING
            node.bale_active=False
            node._update_safety_state();node._update_control_mode()
            self.reply(req,True,'OK','已切回遥控，速度和转角已清零',selected_mode='remote')
        elif req['action']=='reload':
            self.loaded_version=result['path_version'];self.pending=None
            node=self.node
            self.pause()
            node.teleop_active=False
            node.bale_active=False
            self.drive.publish(UInt8(data=node.drive_state))
            node._update_safety_state();node._update_control_mode()
            self.reply(req,True,'OK',('路径重载成功，请开始循迹' if not node.estop_latched and not node.abort_req else '路径加载成功，急停尚未解除，请先处理故障并复位'),path_version=self.loaded_version,
                waypoint_count=result['waypoint_count'],path_file=result.get('path_file',''),
                selected_mode='trajectory',ready_to_start=not node.estop_latched and not node.abort_req)
        elif req['action']=='select':
            self.pending=None
            node=self.node
            self.pause();node.teleop_active=False;node.bale_active=False
            if result.get('path_version'):self.loaded_version=result['path_version']
            self.drive.publish(UInt8(data=node.drive_state))
            node._update_safety_state();node._update_control_mode()
            self.reply(req,True,'OK','已切换循迹模式，请按键g开始循迹',path_version=self.loaded_version,
                selected_mode='trajectory',ready_to_start=not node.estop_latched and not node.abort_req)
        else:
            if self.loaded_version is None and result.get('path_version'):
                self.loaded_version=result['path_version']
            reason=self.start_rejection(req)
            self.pending=None
            if reason:
                self.reply(req,False,'TRAJECTORY_START_REJECTED',reason);return
            node=self.node
            node.teleop_active=False;node.bale_active=False;node.drive_state=node.DS_RUNNING
            node._update_safety_state();node._update_control_mode()
            self.drive.publish(UInt8(data=1))
            self.reply(req,True,'OK','循迹启动已确认',path_version=self.loaded_version,
                waypoint_count=result.get('waypoint_count',0))

    def check_timeout(self):
        if self.pending and time.monotonic()-self.pending[1]>8:
            req,_=self.pending;self.pending=None
            self.reply(req,False,'TRAJECTORY_TIMEOUT','循迹节点未回复，保持暂停')


class BridgePathClient:
    def __init__(self,node):
        self.node=node;self.pending={}
        self.pub=node.create_publisher(String,'/trajectory/control',10)
        node.create_subscription(String,'/vehicle/trajectory_result',self.on_result,10)
    def on_result(self,msg):
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):return
        if not isinstance(data,dict):return
        future=self.pending.get(data.get('request_id'))
        if future is not None and not future.done():future.set_result(data)
    async def request(self,ws,data):
        rid=data.get('request_id',str(uuid.uuid4()));action=data.get('action')
        if action=='toggle':
            action='select' if self.node.teleop_active else 'remote'
            data=dict(data,action=action)
        if action=='start' and not data.get('path_version'):
            data=dict(data,path_version=getattr(self,'loaded_version',None))
        base=dict(type='trajectory_result',request_id=rid,action=action)
        if not isinstance(rid,str) or not 1<=len(rid)<=128 or action not in ('reload','start','remote','select'):
            await ws.send(json.dumps(dict(base,ok=False,code='INVALID_REQUEST',message='无效路径请求'),ensure_ascii=False));return
        if self.pending:
            await ws.send(json.dumps(dict(base,ok=False,code='TRAJECTORY_BUSY',message='另一条路径操作正在执行'),ensure_ascii=False));return
        future=asyncio.get_running_loop().create_future();self.pending[rid]=future
        try:
            req=dict(request_id=rid,action=action)
            if action=='start':req['path_version']=data.get('path_version')
            publish(self.pub,req)
            result=await asyncio.wait_for(future,10)
            if result.get('ok'):
                self.node.zero_teleop();self.node.drive_state=result['drive_state']
                self.node.teleop_active=action=='remote'
                if result.get('path_version'):self.loaded_version=result['path_version']
                self.node.publish_teleop_active()
        except asyncio.TimeoutError:
            result=dict(base,ok=False,code='TRAJECTORY_TIMEOUT',message='执行节点未回复，操作结果未确认')
        finally:self.pending.pop(rid,None)
        try:await ws.send(json.dumps(result,ensure_ascii=False))
        except Exception:pass
