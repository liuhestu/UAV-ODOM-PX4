"""Deterministic one-shot flight execution. All service actions are emitted once."""
import math
from numbers import Real
from dataclasses import dataclass
from uav_core.validation import positive, attitude_target
from mission_executor.shutdown import ThrustStop


@dataclass(frozen=True)
class AttitudeCommand:
    """base_link FLU -> PX4 local ENU quaternion (xyzw), normalized thrust."""
    orientation: tuple
    thrust: float


@dataclass(frozen=True)
class TaskUpdate:
    """Task target, diagnostic phase and completion result; independent of ROS."""
    target: object
    phase: str
    done: bool = False


class ExecutionController:
    TERMINAL = {'DONE', 'BLOCKED', 'ABORTED', 'TAKEN_OVER'}
    def __init__(self, cfg, task):
        self.task=task; self.is_active=False
        self.command_kind=getattr(task,'command_kind','position')
        if self.command_kind not in ('position','attitude'): raise ValueError('invalid task command_kind')
        self.max_attitude_thrust=cfg.get('max_attitude_thrust',0.3)
        positive({'limit':self.max_attitude_thrust},('limit',))
        if self.max_attitude_thrust>1: raise ValueError('invalid max_attitude_thrust')
        if self.command_kind=='attitude':
            attitude_target((0,0,0,1),getattr(task,'peak_thrust',None),self.max_attitude_thrust)
        self.cfg=cfg; self.state='WAIT_SYSTEM'; self.since=0; self.stable=None
        self.thrust_stop=ThrustStop(cfg.get('attitude_ramp_down_seconds',2.0),
                                   cfg.get('attitude_zero_thrust_seconds',0.5))
        self.session=None; self.hold=None; self.target=None; self.reason=''
        self.seen_armed=False; self.preparation_mode=None; self.arm_requested=False
        self.seen_connected=False; self.offboard_requested=False
        positive(cfg, ('prestream_seconds', 'transition_timeout', 'landing_timeout', 'ready_stable_seconds'))
        if type(cfg.get('auto_arm')) is not bool: raise ValueError('auto_arm must be boolean')

    def enter(self, state, now, reason=''):
        self.state=state; self.since=now; self.reason=reason
        self.is_active=False

    @classmethod
    def validate_update(cls, update):
        reserved=cls.TERMINAL | {'WAIT_SYSTEM','PRESTREAM','WAIT_OFFBOARD','WAIT_ARM',
                                'WAIT_LAND_MODE','WAIT_LAND','WAIT_DISARM','ABORT_LAND_MODE','ABORT_LAND',
                                'WAIT_GROUND','ABORT_GROUND','ABORT_DISARM','RAMP_DOWN','ZERO_THRUST'}
        if not isinstance(update,TaskUpdate) or type(update.done) is not bool:
            raise ValueError('task must return TaskUpdate with boolean done')
        if not isinstance(update.phase,str) or not update.phase.strip() or update.phase in reserved:
            raise ValueError('invalid task phase')
        if isinstance(update.target,AttitudeCommand):
            attitude_target(update.target.orientation,update.target.thrust)
            return update
        if len(update.target)!=4 or any((not isinstance(v,Real) or isinstance(v,bool)) or not math.isfinite(v) for v in update.target):
            raise ValueError('task target must contain four finite numbers')
        return update

    def preparation_target(self, local):
        if self.command_kind=='attitude':
            yaw=local[3]
            return AttitudeCommand((0.0,0.0,math.sin(yaw/2),math.cos(yaw/2)),0.0)
        return local

    def task_update(self, update):
        update=self.validate_update(update)
        is_attitude=isinstance(update.target,AttitudeCommand)
        if is_attitude != (self.command_kind=='attitude'):
            raise ValueError('task changed command kind')
        if is_attitude:
            attitude_target(update.target.orientation,update.target.thrust,self.max_attitude_thrust)
        return update

    def stop_attitude(self, now, reason='', abort=False):
        orientation=self.target.orientation if isinstance(self.target,AttitudeCommand) else self.preparation_target(self.hold).orientation
        self.target=AttitudeCommand(orientation,0.0)
        self.enter('ABORT_GROUND' if abort else 'WAIT_GROUND',now,reason)
        return self.target

    def step(self, now, ready, session, fcu, local, landed, service_result=None, service_pending=False,
             offboard_confirmed=False):
        # fcu=(connected, armed, mode); local=(x,y,z,yaw); result=(action, accepted)
        action=None; command=None
        if self.state in self.TERMINAL: return command, action
        connected, armed, mode = fcu if fcu else (False, False, '')
        if connected:
            self.seen_connected=True
        elif self.seen_connected:
            self.enter('ABORTED',now,'FCU telemetry lost; restart Mission Executor for a new mission')
            return None,None
        if connected and armed:
            self.seen_armed=True
        elif connected and self.seen_armed and not armed:
            if self.state in ('WAIT_LAND_MODE','WAIT_LAND','WAIT_DISARM','WAIT_GROUND') and landed==1:
                self.enter('DONE',now)
            elif self.state in ('ABORT_GROUND','ABORT_DISARM') and landed==1:
                self.enter('ABORTED',now,'rig stopped after fault; no automatic restart')
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
                    self.hold=local; self.target=self.preparation_target(local); self.preparation_mode=mode
                    self.enter('PRESTREAM', now)
            else:
                self.stable=None
                self.reason='waiting for fresh ON_GROUND confirmation' if landed!=1 else 'waiting for system readiness and fresh local pose'
            return command, action
        if self.state in ('WAIT_GROUND','ABORT_GROUND','ABORT_DISARM'):
            abort=self.state.startswith('ABORT')
            if mode!='OFFBOARD':
                self.enter('TAKEN_OVER',now,'pilot/FCU mode change'); return None,None
            command=self.target  # Zero thrust; never resume a rig hold after a fault.
            if self.state=='ABORT_DISARM':
                if service_result and service_result[0]=='DISARM' and not service_result[1]:
                    self.enter('ABORTED',now,'DISARM rejected; inspect PX4 report')
                elif now-self.since>self.cfg['transition_timeout']:
                    self.enter('ABORTED',now,'DISARM state timeout')
            elif landed==1 and not service_pending:
                self.enter('ABORT_DISARM' if abort else 'WAIT_DISARM',now,self.reason); action='DISARM'
            elif now-self.since>self.cfg['landing_timeout']:
                self.enter('ABORTED',now,'ground confirmation timeout; no airborne DISARM')
            return (None if self.state in self.TERMINAL else command),action

        if self.state in ('WAIT_LAND_MODE','WAIT_LAND','WAIT_DISARM','ABORT_LAND_MODE','ABORT_LAND'):
            abort=self.state.startswith('ABORT')
            if self.state=='WAIT_DISARM' and isinstance(self.target,AttitudeCommand):
                if armed and mode!='OFFBOARD':
                    self.enter('TAKEN_OVER',now,'pilot/FCU mode change'); return None,None
                command=self.target
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
            return (None if self.state in self.TERMINAL else command), action
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
                if self.command_kind=='attitude':
                    return self.stop_attitude(now,'system readiness lost',abort=True),None
                self.enter('ABORT_LAND_MODE', now, 'system readiness lost'); action='LAND'
            else: self.enter('ABORTED', now, 'system readiness lost')
            return None, action
        if preparing: self.target=self.preparation_target(local)
        if self.state in ('RAMP_DOWN','ZERO_THRUST'):
            thrust,phase,done=self.thrust_stop.step(now)
            self.target=AttitudeCommand(self.target.orientation,thrust)
            if done:
                return self.stop_attitude(now),None
            if phase!=self.state:
                self.enter(phase,now)
            return self.target,None
        command=self.target
        if service_pending: return command,None
        if self.state=='PRESTREAM':
            if now-self.since>=self.cfg['prestream_seconds']:
                self.offboard_requested=True
                self.enter('WAIT_OFFBOARD',now); action='OFFBOARD'
        elif self.state=='WAIT_OFFBOARD':
            if mode=='OFFBOARD' and offboard_confirmed: self.enter('WAIT_ARM',now)
            elif service_result and service_result[0]=='OFFBOARD' and not service_result[1]: self.enter('BLOCKED',now,'OFFBOARD rejected')
            elif now-self.since>self.cfg['transition_timeout']: self.enter('BLOCKED',now,'OFFBOARD state timeout')
        if self.state=='WAIT_ARM':
            if armed:
                if not self.offboard_requested:
                    self.offboard_requested=True
                    self.enter('WAIT_OFFBOARD',now); action='OFFBOARD'
                elif mode=='OFFBOARD':
                    self.hold=local; self.target=self.preparation_target(local)
                    try:
                        update=self.task_update(self.task.start(now,local))
                        self.enter(update.phase,now); self.is_active=True
                    except Exception as exc:
                        if self.command_kind=='attitude':
                            return self.stop_attitude(now,'task failed: '+str(exc),abort=True),None
                        self.enter('ABORT_LAND_MODE',now,'task failed: '+str(exc)); action='LAND'; command=None
                else:
                    self.enter('TAKEN_OVER',now,'OFFBOARD lost after confirmation')
            elif self.cfg['auto_arm']:
                if not self.arm_requested:
                    self.arm_requested=True; self.since=now; action='ARM'
                elif service_result and service_result[0]=='ARM' and not service_result[1]: self.enter('BLOCKED',now,'ARM rejected; inspect PX4 report')
                elif now-self.since>self.cfg['transition_timeout']: self.enter('BLOCKED',now,'ARM state timeout')
            else: self.reason='waiting for actual ARM; automatic ARM requests disabled'
        elif self.is_active and not preparing:
            if not armed:
                self.enter('ABORTED',now,'unexpected disarm'); return None, None
            try:
                previous_target=self.target
                update=self.task_update(self.task.step(now,local))
                self.target=update.target if isinstance(update.target,AttitudeCommand) else tuple(update.target); command=self.target
                if update.phase!=self.state:
                    self.enter(update.phase,now); self.is_active=True
                if update.done:
                    if self.command_kind=='attitude':
                        # Start from the last emitted thrust, even if a task's
                        # completion update jumps directly to zero.
                        self.target=previous_target
                        self.thrust_stop.start(now,self.target.thrust)
                        self.enter('RAMP_DOWN',now)
                        command=self.target
                    else:
                        self.enter('WAIT_LAND_MODE',now); action='LAND'
            except Exception as exc:
                if self.command_kind=='attitude':
                    return self.stop_attitude(now,'task failed: '+str(exc),abort=True),None
                self.enter('ABORT_LAND_MODE',now,'task failed: '+str(exc)); action='LAND'; command=None
        if self.state in self.TERMINAL: command=None
        return command, action


class FcuGuard:
    def __init__(self, system_id=1, component_id=1):
        if any(type(value) is not int or not 1 <= value <= 255 for value in (system_id,component_id)):
            raise ValueError('invalid FCU MAVLink identity')
        self.system_id = system_id
        self.component_id = component_id
        self.connected_once = False
        self.boot_clocks = {}
        self.fault = ''

    def observe_connection(self, connected):
        if connected:
            self.connected_once = True
        elif self.connected_once:
            self.fault = self.fault or 'FCU connection interrupted; restart Mission Executor for a new mission'

    def observe_boot(self, system_id, component_id, message_id, boot_ms):
        if (system_id, component_id) != (self.system_id, self.component_id):
            return
        if message_id not in (2, 30, 31, 32) or not 0 <= boot_ms <= 0xffffffff:
            return
        previous = self.boot_clocks.get(message_id)
        # Separate clocks avoid cross-stream ordering. Ignore small packet
        # reordering and the normal uint32 rollover after about 49.7 days.
        if previous is not None and boot_ms < previous:
            rollover = previous >= 0xffff0000 and boot_ms < 0x10000
            if previous - boot_ms > 1000 and not rollover:
                self.fault = self.fault or 'FCU boot clock reset; restart Mission Executor for a new mission'
            if not rollover:
                return
        self.boot_clocks[message_id] = boot_ms
