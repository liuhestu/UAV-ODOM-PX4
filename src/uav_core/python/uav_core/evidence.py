"""Parse a fixed read-only PX4 listener protocol; missing fields fail closed."""
import re

TOPICS = ('vehicle_visual_odometry', 'estimator_aid_src_ev_pos', 'estimator_aid_src_ev_hgt', 'vehicle_local_position')
RESETS = ('xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter','heading_reset_counter')


def parse_listener(topic, text):
    text = re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]', '', text).replace('\r','')
    # Require topic output, not just echoed command text.
    marker = re.search(r'(?:TOPIC:\s*|^)' + re.escape(topic) + r'(?:\s|$)', text, re.MULTILINE)
    if not marker: raise ValueError('missing topic output: '+topic)
    result={}
    for name, val in re.findall(r'^\s*([a-z_]+):\s*([-+0-9.eE]+|True|False|true|false)\b', text[marker.end():], re.MULTILINE):
        if name in result: raise ValueError('ambiguous repeated field: '+name)
        boolean_fields={'fused','innovation_rejected','xy_valid','z_valid','v_xy_valid','v_z_valid'}
        if name in boolean_fields:
            if val.lower() not in ('true','false','0','1'):
                raise ValueError('invalid boolean: '+name)
            result[name]=val.lower() in ('true','1')
        else:
            result[name] = float(val)
    return result


def evaluate_listener(samples, max_age):
    vo,pos,hgt,local = [samples[t] for t in TOPICS]
    now=local['timestamp']
    if now<=0: raise ValueError('invalid PX4 clock')
    def fresh(t): return 0<=now-t<=max_age*1e6
    received=fresh(vo['timestamp']) and fresh(vo['timestamp_sample'])
    fused=all(a['fused'] is True and a['innovation_rejected'] is False and
              a['estimator_instance']==0 and fresh(a['timestamp']) and fresh(a['time_last_fuse'])
              for a in (pos,hgt))
    valid=all(local[k] is True for k in ('xy_valid','z_valid','v_xy_valid','v_z_valid'))
    reset=0
    for i,k in enumerate(RESETS):
        value=int(local[k])
        if not 0<=value<=255: raise ValueError('invalid reset counter')
        reset |= value << (i*8)
    return received, fused and valid, reset
