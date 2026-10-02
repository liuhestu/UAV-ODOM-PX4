"""Resolve task/configuration and guard one Mission Executor per ROS master."""
import fcntl
import hashlib
import importlib.util
import os
import sys
from pathlib import Path
from dataclasses import dataclass
import tempfile
import yaml
from mission_executor.execution import ExecutionController
from uav_core.validation import positive


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


def resolve_source(package, source):
    root = (Path(package) / 'src/mission').resolve()
    requested = Path(source)
    if requested.is_absolute():
        path = requested
    elif str(requested).startswith('src/mission/'):
        path = Path(package) / requested
    else:
        path = root / requested
    if not path.suffix:
        path = path.with_suffix('.py')
    path = path.resolve()
    if root not in path.parents or path.suffix != '.py' or not path.is_file():
        raise ValueError('mission_source must be a Python task inside ' + str(root))
    return path


@dataclass(frozen=True)
class LoadedMission:
    source_path: Path
    mission_config_path: Path
    executor_config_path: Path
    executor_config: dict
    mission_config: dict
    task: object


def read_config(path):
    cfg = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(cfg, dict):
        raise ValueError('configuration must be a YAML mapping: ' + str(path))
    return cfg


def load_task(package, source='takeoff_hover_land', config='', executor_config='',
              auto_arm='', arm_method='', warn=lambda message: None):
    package = Path(package)
    source_path = resolve_source(package, source)
    config_path = Path(config) if config else package / 'config/mission' / (source_path.stem + '.yaml')
    if config and not config_path.is_absolute():
        config_path = package / config_path if str(config_path).startswith('config/') else package / 'config/mission' / config_path
    config_path = config_path.resolve()
    executor_path = Path(executor_config) if executor_config else package / 'config/mission_executor.yaml'
    if executor_config and not executor_path.is_absolute():
        executor_path = package / executor_path if str(executor_path).startswith('config/') else package / 'config' / executor_path
    executor_path = executor_path.resolve()
    cfg = read_config(executor_path)
    cfg.update(auto_arm_override=auto_arm, arm_method_override=arm_method)
    cfg = normalize_config(cfg, warn)
    positive(cfg, ('rate_hz','status_timeout','state_timeout','telemetry_timeout','max_local_position_step'))
    mission_cfg = read_config(config_path)
    # Task configuration cannot override or silently duplicate flight execution policy.
    common_keys = set(cfg) | {'auto_arm', 'arm_method', 'auto_arm_override', 'arm_method_override'}
    if common_keys.intersection(mission_cfg):
        raise ValueError('execution settings belong in executor_config: ' + ', '.join(sorted(common_keys.intersection(mission_cfg))))
    spec = importlib.util.spec_from_file_location('uav_task_' + hashlib.sha256(str(source_path).encode()).hexdigest(), source_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(spec.name, None)
        raise
    factory = getattr(module, 'create_task', None)
    if not callable(factory):
        raise ValueError('task module must define create_task(config)')
    task = factory(mission_cfg)
    if not all(callable(getattr(task, name, None)) for name in ('start', 'step')):
        raise ValueError('task must implement start(now, origin) and step(now, local)')
    ExecutionController(cfg, task)  # Validate settings before creating flight services/publishers.
    # Validate identity before entering ROS runtime, too.
    from mission_executor.execution import FcuGuard
    FcuGuard(cfg.get('fcu_system_id', 1), cfg.get('fcu_component_id', 1))
    return LoadedMission(source_path, config_path, executor_path, cfg, mission_cfg, task)


class ExecutorLock:
    def __init__(self, master_uri=None):
        uri = (master_uri or os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')).rstrip('/')
        # Treat local aliases as the same master endpoint.
        from urllib.parse import urlparse
        parsed = urlparse(uri)
        host = parsed.hostname
        if host in ('localhost','127.0.0.1','::1'):
            host = 'localhost'
        key = '{}:{}'.format(host, parsed.port or 11311)
        # Keep the legacy lock namespace to exclude an older Commander process.
        path = Path(tempfile.gettempdir()) / ('uav_commander_' + hashlib.sha256(key.encode()).hexdigest() + '.lock')
        self.handle = path.open('a')
        try:
            fcntl.flock(self.handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self.handle.close()
            raise RuntimeError('Mission Executor already running for this ROS master') from exc

    def close(self):
        self.handle.close()
