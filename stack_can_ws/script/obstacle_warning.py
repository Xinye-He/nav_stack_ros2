"""Display-only four-level distance warning. Does not change health or control."""
import math
CHANNELS=('front_left','front_right','rear_left','rear_right')

def summarize(values, received_at, now, timeout=2.0, threshold=0.5, warning_threshold=2.0):
    age=None if received_at is None else max(0.0,now-received_at)
    fresh=age is not None and age <= timeout
    distances={name:None for name in CHANNELS}
    if fresh and values is not None:
        for name,value in zip(CHANNELS,values):
            if math.isfinite(value) and value > 0:
                distances[name]=float(value)
    states={name:('unknown' if value is None else
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
