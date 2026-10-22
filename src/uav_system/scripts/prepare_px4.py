#!/usr/bin/env python3
"""One-shot ground preparation. Never run from an in-flight watchdog."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import termios
import time

DEVICE = '/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00'


def guard(device):
    # Check actual processes rather than trusting PID files or a stale ROS graph.
    for item in Path('/proc').iterdir():
        if not item.name.isdigit() or int(item.name) == os.getpid():
            continue
        try:
            args = (item / 'cmdline').read_bytes().split(b'\0')
        except (OSError, PermissionError):
            continue
        if any(Path(a.decode(errors='replace')).name == 'mission_executor_node.py'
               for a in args if a):
            raise RuntimeError('Mission Executor is running; recovery prohibited')
    result = subprocess.run(['fuser', str(device)], capture_output=True, text=True)
    owners = set(result.stdout.split())
    if result.returncode == 0 and owners - {str(os.getpid())}:
        raise RuntimeError('Serial device occupied: ' + result.stdout.strip())
    if result.returncode not in (0, 1):
        raise RuntimeError('Cannot verify serial ownership: ' + result.stderr.strip())


def observe(conn, duration, stable_seconds, clock=time.monotonic):
    from pymavlink import mavutil
    deadline = clock() + duration
    first = last = None
    count = 0
    while clock() < deadline:
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=0.2)
        now = clock()
        if not msg or msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1:
            if last is not None and now - last > 2.5:
                first = last = None
            continue
        if msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
            continue
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise RuntimeError('FCU armed: reboot and preparation prohibited')
        if last is None or now - last > 2.5:
            first = now
        last = now
        count += 1
        if now - first >= stable_seconds:
            return True, count
    return False, count


def recover(open_connection, precheck, ground_confirmed, emit, attempts=3,
            stable_seconds=10.0, wait=time.sleep):
    for attempt in range(attempts + 1):
        conn = None
        try:
            precheck()
            conn = open_connection()
            ok, count = observe(conn, stable_seconds + 5, stable_seconds)
            emit('heartbeat_check', attempt=attempt, stable=ok, heartbeat_count=count)
            if ok:
                return
            if not ground_confirmed:
                raise RuntimeError('No stable telemetry; ground confirmation required for reboot')
            if attempt == attempts:
                raise RuntimeError('FCU recovery exhausted; preparation stopped')
            precheck()  # Recheck Mission/ownership immediately before reboot.
            # No mode, ARM or parameter commands. A missing ACK is not success.
            from pymavlink import mavutil
            conn.mav.command_long_send(1, 1,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0, 1, 0, 0, 0, 0, 0, 0)
            emit('reboot_sent', attempt=attempt + 1)
        except (OSError,) as exc:
            # Stop on serial I/O errors; never turn an armed observation,
            # ownership conflict, or failed precheck into a recovery retry.
            raise RuntimeError('Serial I/O failed; stop preparation: ' + str(exc)) from exc
        finally:
            if conn is not None:
                conn.close()
        wait(15)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default=DEVICE)
    parser.add_argument('--ground-confirmed', action='store_true',
                        help='Operator confirms disarmed ground preparation; permits reboot without RX')
    parser.add_argument('--start-base', action='store_true',
                        help='After stable heartbeats start real OpenVINS/base chain; never Mission')
    parser.add_argument('--reuse-source', action='store_true')
    parser.add_argument('--log', type=Path, required=True)
    args = parser.parse_args()
    args.log.parent.mkdir(parents=True, exist_ok=True)
    def emit(event, **fields):
        row = dict(time=time.time(), event=event, **fields)
        with args.log.open('a') as output:
            output.write(json.dumps(row) + '\n')
        print(json.dumps(row), flush=True)
    lock = open('/tmp/uav_px4_prepare.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        from pymavlink import mavutil
        def connect():
            conn = mavutil.mavlink_connection(args.device, baud=57600,
                        source_system=246, source_component=190, autoreconnect=False)
            try:
                fcntl.ioctl(conn.port.fileno(), termios.TIOCEXCL)
                conn.port.write_timeout = 1.0
            except Exception:
                conn.close()
                raise
            return conn
        if args.start_base:
            import xmlrpc.client
            import socket
            socket.setdefaulttimeout(3)
            master = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'))
            try:
                code, _, groups = master.getSystemState('/px4_prepare')
            except OSError:
                groups = []  # roslaunch will create a new master
            blocked = {'/mission_executor', '/mavros', '/control_backend',
                       '/flight_supervisor', '/state_adapter', '/state_source_manager'}
            if not args.reuse_source:
                blocked.update({'/ov_msckf', '/camera/realsense2_camera'})
            nodes = {node for group in groups for _, names in group for node in names}
            if nodes & blocked:
                raise RuntimeError('Existing chain; do not launch duplicate nodes: ' + str(sorted(nodes & blocked)))
        emit('prepare_started', device=args.device, ground_confirmed=args.ground_confirmed)
        recover(connect, lambda: guard(args.device), args.ground_confirmed, emit)
        emit('communication_ready')
        if args.start_base:
            # New process/session after any reboot; no old Mission can resume.
            command = ['roslaunch', 'uav_system', 'uav_system.launch',
                       'state_source:=openvins',
                       'start_source:=' + ('false' if args.reuse_source else 'true'),
                       'start_mavros:=true', 'start_ev_observer:=true',
                       'output_enabled:=true', 'start_mission_executor:=false']
            emit('starting_base', command=command)
            os.execvp(command[0], command)
    except Exception as exc:
        emit('prepare_failed', error=str(exc))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
