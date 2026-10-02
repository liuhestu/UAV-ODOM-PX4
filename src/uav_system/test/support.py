import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
WORKSPACE=ROOT.parents[1]
sys.path.insert(0,str(ROOT/'src/support'))
sys.path.insert(0,str(ROOT/'src'))


def takeoff_task(config):
    import importlib.util
    spec = importlib.util.spec_from_file_location('test_takeoff_task', ROOT/'src/mission/takeoff_hover_land.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    import yaml
    task_config = yaml.safe_load((ROOT/'config/mission/takeoff_hover_land.yaml').read_text())
    return module.create_task(task_config)
