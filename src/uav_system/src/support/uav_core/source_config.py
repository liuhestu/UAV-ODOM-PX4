import re
import math
from pathlib import Path
import yaml
from .geometry import Adapter


def load_source(path):
    with open(path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict) or cfg.get('schema_version') != 1:
        raise ValueError('unsupported source schema')
    if not isinstance(cfg['source']['simulated'], bool) or not isinstance(cfg['extrinsic']['calibrated'], bool):
        raise ValueError('simulated/calibrated must be booleans')
    for item in cfg['source']['launches']:
        if not re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_]*', item['package']):
            raise ValueError('invalid package')
        if not item['file'].endswith('.launch') or '..' in Path(item['file']).parts:
            raise ValueError('invalid launch file')
        if not isinstance(item.get('args', {}), dict):
            raise ValueError('launch args must be a mapping')
    nodes = cfg['source'].get('nodes', [])
    if not isinstance(nodes, list):
        raise ValueError('source nodes must be a list')
    names = set()
    for item in nodes:
        if not isinstance(item, dict):
            raise ValueError('source node must be a mapping')
        for key in ('package', 'name'):
            if not isinstance(item.get(key), str) or not re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_]*', item[key]):
                raise ValueError('invalid source node ' + key)
        if item['name'] in names:
            raise ValueError('duplicate source node name')
        names.add(item['name'])
        if not isinstance(item.get('executable'), str) or not re.fullmatch(r'[a-zA-Z0-9_][a-zA-Z0-9_.-]*', item['executable']):
            raise ValueError('invalid source node executable')
        params = item.get('params', {})
        if not isinstance(params, dict):
            raise ValueError('source node params must be a mapping')
        for key, value in params.items():
            if not isinstance(key, str) or not re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_]*', key):
                raise ValueError('invalid source node parameter name')
            if type(value) not in (str, bool, int, float) or (type(value) is float and not math.isfinite(value)):
                raise ValueError('source node parameters must be finite scalar values')
    for key in ('world_frame', 'body_frame', 'topic'):
        if not cfg['input'][key]:
            raise ValueError('input frame/topic must be explicit')
    for key in ('stale_timeout','max_position_step','max_rotation_step_rad'):
        if not math.isfinite(cfg['adapter'][key]):
            raise ValueError('nonfinite adapter limit')
    if cfg['adapter']['max_rotation_step_rad'] <= 0:
        raise ValueError('invalid rotation limit')
    if cfg['adapter']['stale_timeout'] <= 0 or cfg['adapter']['max_position_step'] <= 0:
        raise ValueError('invalid adapter limits')
    Adapter(cfg)
    return cfg
