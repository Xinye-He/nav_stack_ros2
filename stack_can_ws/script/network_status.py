"""Network status is separate from peripheral health and sticky device faults."""

def summarize_network(control, control_fresh, link_ok, link_fresh, event):
    reason = control.get('estop_reason', '')
    network_estop = control_fresh and control.get('safety_state') == 'ESTOP' and reason in ('network_disconnected','network_heartbeat_timeout','network_unconfirmed')
    if network_estop:
        return dict(state='connected' if link_ok and link_fresh else 'disconnected', severity='emergency',
                    message='9010遥控断链急停已锁存，重连后需手动复位', last_event=event)
    if link_ok and link_fresh:
        return dict(state='connected', severity='none', message='9010控制链路正常', last_event=event)
    if not event:
        return dict(state='waiting_connection', severity='none', message='等待上位机控制连接', last_event=None)
    message = '控制链路未连接'
    if control_fresh and control.get('safety_state') == 'READY' and control.get('control_mode') == 'TRAJ':
        message = '循迹模式上位机断链，仅告警，循迹继续运行'
    elif event.get('action') in ('warning', 'neutral'):
        message = event.get('message', message)
    return dict(state='disconnected' if link_fresh else 'unknown', severity='warning',
                message=message if link_fresh else '控制链路状态未更新', last_event=event)
