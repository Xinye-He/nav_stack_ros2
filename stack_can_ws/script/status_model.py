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
    if control.get('safety_state') != 'READY' or control.get('drive_state') != 1:
        return None
    if control.get('control_mode') == 'TRAJ':
        return 'trajectory'
    if (control.get('control_mode') == 'TELEOP'
            and control.get('command_source') == 'websocket_9010' and link_ok):
        return 'remote'
    return None
