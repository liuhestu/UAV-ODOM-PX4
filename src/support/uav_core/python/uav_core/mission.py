"""Deterministic one-shot mission. All service actions are emitted once."""
import math


class Mission:
    TERMINAL = {'DONE', 'BLOCKED', 'ABORTED', 'TAKEN_OVER'}
    def __init__(self, cfg):
        self.cfg=cfg; self.state='WAIT_SYSTEM'; self.since=0; self.stable=None
        self.session=None; self.hold=None; self.target=None; self.reason=''
        for name in ('height','climb_rate','hover_seconds','prestream_seconds','transition_timeout', 'flight_timeout', 'landing_timeout', 'ready_stable_seconds'):
            value=cfg[name]
            if not math.isfinite(value) or value<=0: raise ValueError('invalid '+name)
        if cfg['arm_method'] not in ('auto','manual'): raise ValueError('unknown arm_method')

    def enter(self, state, now, reason=''):
        self.state=state; self.since=now; self.reason=reason

    def step(self, now, ready, session, fcu, local, landed, service_result=None, service_pending=False):
        # fcu=(connected, armed, mode); local=(x,y,z,yaw); result=(action, accepted)
        action=None; command=None
        if self.state in self.TERMINAL: return command, action
        connected, armed, mode = fcu if fcu else (False, False, '')
        if self.state=='WAIT_SYSTEM':
            if armed:
                self.enter('BLOCKED', now, 'vehicle already armed at commander startup')
            elif ready and connected and local is not None and landed==1:
                if self.stable is None: self.stable=now
                if now-self.stable>=self.cfg['ready_stable_seconds']:
                    self.session=session; self.hold=local; self.target=local
                    self.enter('PRESTREAM', now)
            else: self.stable=None
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
        if armed and self.state=='PRESTREAM':
            self.enter('TAKEN_OVER', now, 'unexpected arming before requested OFFBOARD')
            return None, None
        if self.state not in ('PRESTREAM','WAIT_OFFBOARD') and mode!='OFFBOARD':
            self.enter('TAKEN_OVER', now, 'pilot/FCU mode change'); return None, None
        if not connected or not ready or session!=self.session or local is None:
            if service_pending:
                self.enter('ABORTED',now,'readiness lost during service request; inspect actual FCU state')
                return None,None
            if connected and armed and mode=='OFFBOARD':
                self.enter('ABORT_LAND_MODE', now, 'system readiness lost'); action='LAND'
            else: self.enter('ABORTED', now, 'system readiness lost')
            return None, action
        command=self.target
        if service_pending: return command,None
        if self.state=='PRESTREAM':
            if now-self.since>=self.cfg['prestream_seconds']:
                self.enter('WAIT_OFFBOARD',now); action='OFFBOARD'
        elif self.state=='WAIT_OFFBOARD':
            if mode=='OFFBOARD':
                self.enter('WAIT_ARM',now)
                if self.cfg['arm_method']=='auto': action='ARM'
            elif service_result and service_result[0]=='OFFBOARD' and not service_result[1]: self.enter('BLOCKED',now,'OFFBOARD rejected')
            elif now-self.since>self.cfg['transition_timeout']: self.enter('BLOCKED',now,'OFFBOARD state timeout')
        elif self.state=='WAIT_ARM':
            if armed: self.enter('TAKEOFF',now)
            elif service_result and service_result[0]=='ARM' and not service_result[1]: self.enter('BLOCKED',now,'ARM rejected; inspect PX4 report')
            elif now-self.since>self.cfg['transition_timeout']: self.enter('BLOCKED',now,'ARM state timeout')
        elif self.state in ('TAKEOFF','HOVER'):
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
