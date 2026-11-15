"""Display-only four-level distance warning. Does not change health or control."""
import math
from collections import deque
from datetime import datetime, timezone
CHANNELS=('front_left','front_right','rear_left','rear_right')


class ObstacleHistory:
    """Record new per-channel warning/alarm transitions, independently of live state."""
    def __init__(self, limit=50):
        self.events = deque(maxlen=limit)
        self.previous = {name: 'unknown' for name in CHANNELS}

    def clear(self):
        # Keep current channel states: an ongoing obstacle remains visible live,
        # but clearing history alone must not recreate the same event every tick.
        self.events.clear()

    def update(self, current):
        for name in CHANNELS:
            state = current['channel_states'][name]
            if state != self.previous[name] and state in ('warning', 'alarm'):
                self.events.append({'timestamp': datetime.now(timezone.utc).isoformat(),
                                    'channel': name, 'state': state,
                                    'distance_m': current['distances_m'][name]})
            self.previous[name] = state
        return list(self.events)

def summarize(values, received_at, now, timeout=2.0, threshold=1.0, warning_threshold=2.0):
    age=None if received_at is None else max(0.0,now-received_at)
    fresh=age is not None and age <= timeout
    distances={name:None for name in CHANNELS}
    if fresh and values is not None:
        for name,value in zip(CHANNELS,values):
            if math.isfinite(value) and value >= 0:
                distances[name]=float(value)
    states={name:('unknown' if value is None or value <= 0 else
                  'alarm' if value < threshold else
                  'warning' if value <= warning_threshold else 'clear')
            for name,value in distances.items()}
    alarms=[name for name in CHANNELS if states[name]=='alarm']
    warnings=[name for name in CHANNELS if states[name]=='warning']
    unknown=[name for name in CHANNELS if states[name]=='unknown']
    state='alarm' if alarms else 'warning' if warnings else 'unknown' if unknown else 'clear'
    return {'state':state, 'alarm':True if alarms else (None if unknown else False),
            'warning':True if alarms or warnings else (None if unknown else False),
            'distances_m':distances, 'channel_states':states,
            'alarm_threshold_m':threshold, 'warning_threshold_m':warning_threshold,
            'alarm_channels':alarms, 'warning_channels':warnings, 'unknown_channels':unknown,
            'data_age_ms':None if age is None else int(age*1000)}
