"""Fail-closed checks for an operator ESTOP reset that must never select RUN."""
def rejection(state, now):
    if state['remote_req']:
        return '实体遥控正在接管'
    if state['abort_req'] or state['estop_reason'] not in ('', 'drive_cmd'):
        return '存在保护/断联 abort 锁存，来源尚未确认，不允许用作业测试复位覆盖'
    for name, value, stamp in state['guards']:
        if stamp is None or now - stamp > 2.0 or not value:
            return name + '未就绪或状态超时'
    return ''
