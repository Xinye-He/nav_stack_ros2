"""Pure device status rules; no ROS or control side effects."""


def peripheral_status(checks, now, timeout, starting=False):
    result = {}
    for component, items in checks.items():
        errors = []
        ages = []
        for code, label, last_valid, invalid in items:
            age = None if last_valid is None else max(0.0, now - last_valid)
            ages.append(age)
            if invalid:
                errors.append({'code': code + '_INVALID', 'message': label + '数据无效：' + invalid})
            elif age is None or age > timeout:
                errors.append({'code': code + '_TIMEOUT', 'message': label + '未收到有效数据' if age is None else label + '数据超时'})
        state = 'ready' if not errors else ('starting' if starting else 'error')
        result[component] = {'state': state, 'errors': errors if state == 'error' else [],
                             'data_age_ms': None if any(a is None for a in ages) else int(max(ages) * 1000)}
    return result


def vehicle_state(control, link_ok):
    if control.get('drive_state') == 0:
        return 'Neutral'
    if control.get('safety_state') != 'READY' or control.get('drive_state') != 1:
        return None
    if control.get('control_mode') == 'TRAJ':
        return 'trajectory'
    if (control.get('control_mode') == 'TELEOP'
            and control.get('command_source') == 'websocket_9010' and link_ok):
        return 'remote'
    return None



def fault_presentation(control, fresh, events, lifecycle_message=''):
    if not fresh:
        return {'active': False, 'code': 'STATUS_STALE', 'message': '执行器状态超时', 'sources': []}
    if control.get('safety_state') != 'ESTOP':
        return {'active': False, 'code': '', 'message': '', 'sources': []}
    code = control.get('estop_reason') or 'unknown'
    names = {'drive_cmd': '上位机下发急停指令',
             'network_disconnected': '紧急触发：上位机控制连接断开',
             'network_heartbeat_timeout': '紧急触发：上位机控制心跳超时',
             'network_unconfirmed': '紧急触发：网络故障处置未确认',
             'abort': '紧急触发：保护中止（共享abort信号）'}
    sources = []
    # Associate evidence only with the latest observed transition to abort ESTOP.
    estop = next((e for e in reversed(events) if e.get('source') == 'stack_can_executor'
                  and 'safety=ESTOP' in e.get('message', '') and 'estop_reason=abort' in e.get('message', '')), None)
    if code == 'abort' and estop:
        stamp = (estop.get('ros_stamp') or {}).get('sec')
        for e in events:
            when = (e.get('ros_stamp') or {}).get('sec')
            if stamp and when and abs(when-stamp) <= 3 and e.get('evidence') == 'trigger_report':
                sources.append({'source': e.get('source'), 'message': e.get('message')})
    message = names.get(code, '紧急触发：'+code)
    if sources:
        message += '；关联触发记录：'+'；'.join(x['source']+'：'+x['message'] for x in sources)
    elif code == 'abort':
        message += '；具体来源未记录，不能判断唯一原因'
    return {'active': True, 'code': code, 'message': message, 'sources': sources}
