"""Deterministic one-shot mission. All service actions are emitted once."""
import math


def normalize_config(cfg, warn=lambda message: None):
    """Resolve YAML and explicit launch overrides without hiding conflicts."""
    cfg = dict(cfg)
    new_override = cfg.pop('auto_arm_override', '')
    old_override = cfg.pop('arm_method_override', '')
    has_new = 'auto_arm' in cfg
    has_old = 'arm_method' in cfg
    if has_new and has_old:
        raise ValueError('configuration defines both auto_arm and arm_method')
    if new_override != '' and old_override != '':
        raise ValueError('launch defines both auto_arm and arm_method')
    if has_old and new_override != '':
        raise ValueError('legacy arm_method YAML cannot mix with auto_arm launch override')
    if has_new and type(cfg['auto_arm']) is not bool:
        raise ValueError('auto_arm must be a YAML boolean')
    if has_old and cfg['arm_method'] not in ('auto', 'manual'):
        raise ValueError('unknown arm_method')
    if old_override != '' or has_old:
        old = old_override if old_override != '' else cfg['arm_method']
        if old not in ('auto', 'manual'):
            raise ValueError('unknown arm_method')
        warn('arm_method is deprecated; use auto_arm: true/false')
        value = old == 'auto'
    elif new_override != '':
        if new_override not in ('true', 'false'):
            raise ValueError('auto_arm launch argument must be true or false')
        value = new_override == 'true'
    else:
        value = cfg.get('auto_arm', False)
    cfg.pop('arm_method', None)
    cfg['auto_arm'] = value
    return cfg


class Mission:
    TERMINAL = {'DONE', 'BLOCKED', 'ABORTED', 'TAKEN_OVER'}
    def __init__(self, cfg):
        self.cfg=cfg; self.state='WAIT_SYSTEM'; self.since=0; self.stable=None
        self.session=None; self.hold=None; self.target=None; self.reason=''
        self.seen_armed=False; self.preparation_mode=None; self.arm_requested=False
        for name in ('height','climb_rate','hover_seconds','prestream_seconds','transition_timeout', 'flight_timeout', 'landing_timeout', 'ready_stable_seconds'):
            value=cfg[name]
            if not math.isfinite(value) or value<=0: raise ValueError('invalid '+name)
        if type(cfg.get('auto_arm')) is not bool: raise ValueError('auto_arm must be boolean')

    def enter(self, state, now, reason=''):
        self.state=state; self.since=now; self.reason=reason

    def step(self, now, ready, session, fcu, local, landed, service_result=None, service_pending=False):
        # fcu=(connected, armed, mode); local=(x,y,z,yaw); result=(action, accepted)
        action=None; command=None
        if self.state in self.TERMINAL: return command, action
        connected, armed, mode = fcu if fcu else (False, False, '')
        if connected and armed:
            self.seen_armed=True
        elif connected and self.seen_armed and not armed:
            if self.state in ('WAIT_LAND_MODE','WAIT_LAND','WAIT_DISARM') and landed==1:
                self.enter('DONE',now)
            else:
                self.enter('ABORTED',now,'unexpected disarm')
            return None,None
        if self.state=='WAIT_SYSTEM':
            if session!=self.session:
                self.stable=None; self.session=session
            if ready and connected and local is not None and landed==1:
                self.reason=''
                if self.stable is None: self.stable=now
                if now-self.stable>=self.cfg['ready_stable_seconds']:
                    self.hold=local; self.target=local; self.preparation_mode=mode
                    self.enter('PRESTREAM', now)
            else:
                self.stable=None
                self.reason='waiting for fresh ON_GROUND confirmation' if landed!=1 else 'waiting for system readiness and fresh local pose'
            return command, action
        if self.state in ('WAIT_LAND_MODE','WAIT_LAND','WAIT_DISARM','ABORT_LAND_MODE','ABORT_LAND'):
            abort=self.state.startswith('ABORT')
            if not connected:
                self.enter('ABORTED', now, 'FCU telemetry lost; PX4 configured failsafe owns response')
            elif mode not in ('OFFBOARD','AUTO.LAND') and armed:
                self.enter('TAKEN_OVER', now, 'pilot/FCU mode change')
            elif self.state in ('WAIT_LAND_MODE','ABORT_LAND_MODE'):
                if mode=='AUTO.LAND': self.enter('ABORT_LAND' if abort else 'WAIT_LAND', now)
                elif service_result and service_result[0]=='LAND' and not service_result[1]: self.enter('ABORTED', now, 'LAND request rejected')
                elif now-self.since>self.cfg['transition_timeout']: self.enter('ABORTED', now, 'LAND mode timeout')
                elif not abort and ready: command=self.target
            elif self.state=='WAIT_DISARM':
                if not armed: self.enter('DONE', now)
                elif service_result and service_result[0]=='DISARM' and not service_result[1]: self.enter('ABORTED', now, 'DISARM rejected; inspect PX4 report')
                elif now-self.since>self.cfg['transition_timeout']: self.enter('ABORTED', now, 'DISARM state timeout')
            elif landed==1:
                if abort: self.enter('ABORTED', now, 'abort landed; no automatic mission restart')
                elif not armed: self.enter('DONE', now)
                else:
                    self.enter('WAIT_DISARM', now); action='DISARM'
            elif now-self.since>self.cfg['landing_timeout']:
                self.enter('ABORTED', now, 'landing timeout; PX4/pilot owns response')
            return command, action
        preparing=self.state in ('PRESTREAM','WAIT_OFFBOARD','WAIT_ARM')
        if preparing and landed!=1:
            self.enter('ABORTED',now,'ground confirmation lost before takeoff')
            return None,None
        if preparing:
            if connected and mode not in (self.preparation_mode,'OFFBOARD'):
                self.enter('TAKEN_OVER',now,'pilot/FCU mode change'); return None,None
            if connected and mode=='OFFBOARD': self.preparation_mode='OFFBOARD'
        elif connected and mode!='OFFBOARD':
            self.enter('TAKEN_OVER', now, 'pilot/FCU mode change'); return None, None
        if not connected or not ready or session!=self.session or local is None:
            if service_pending:
                self.enter('ABORTED',now,'readiness lost during service request; inspect actual FCU state')
                return None,None
            if not preparing and connected and armed and mode=='OFFBOARD':
                self.enter('ABORT_LAND_MODE', now, 'system readiness lost'); action='LAND'
            else: self.enter('ABORTED', now, 'system readiness lost')
            return None, action
        if preparing: self.target=local
        command=self.target
        if service_pending: return command,None
        if self.state=='PRESTREAM':
            if now-self.since>=self.cfg['prestream_seconds']:
                if mode=='OFFBOARD':
                    self.enter('WAIT_ARM',now)
                else:
                    self.enter('WAIT_OFFBOARD',now); action='OFFBOARD'
        elif self.state=='WAIT_OFFBOARD':
            if mode=='OFFBOARD': self.enter('WAIT_ARM',now)
            elif service_result and service_result[0]=='OFFBOARD' and not service_result[1]: self.enter('BLOCKED',now,'OFFBOARD rejected')
            elif now-self.since>self.cfg['transition_timeout']: self.enter('BLOCKED',now,'OFFBOARD state timeout')
        if self.state=='WAIT_ARM':
            if armed:
                self.hold=local; self.target=local
                self.enter('TAKEOFF',now)
            elif self.cfg['auto_arm']:
                if not self.arm_requested:
                    self.arm_requested=True; self.since=now; action='ARM'
                elif service_result and service_result[0]=='ARM' and not service_result[1]: self.enter('BLOCKED',now,'ARM rejected; inspect PX4 report')
                elif now-self.since>self.cfg['transition_timeout']: self.enter('BLOCKED',now,'ARM state timeout')
            else: self.reason='waiting for remote-control ARM'
        elif self.state in ('TAKEOFF','HOVER') and not preparing:
            if not armed:
                self.enter('ABORTED',now,'unexpected disarm'); return None, None
            x,y,z,yaw=self.hold
            target_z=z+min(self.cfg['height'], self.cfg['climb_rate']*(now-self.since)) if self.state=='TAKEOFF' else z+self.cfg['height']
            self.target=(x,y,target_z,yaw); command=self.target
            if self.state=='TAKEOFF':
                if abs(local[2]-(z+self.cfg['height']))<self.cfg['height_tolerance'] and target_z>=z+self.cfg['height']:
                    self.enter('HOVER',now)
                elif now-self.since>self.cfg['flight_timeout']:
                    self.enter('ABORT_LAND_MODE',now,'takeoff timeout'); action='LAND'; command=None
            elif now-self.since>=self.cfg['hover_seconds']:
                self.enter('WAIT_LAND_MODE',now); action='LAND'
        if self.state in self.TERMINAL: command=None
        return command, action
