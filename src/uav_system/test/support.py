import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
WORKSPACE=ROOT.parents[1]
sys.path.insert(0,str(ROOT/'src/support'))
sys.path.insert(0,str(ROOT/'src'))


FIXTURES = ROOT / 'test/fixtures'


def executor_config():
    """Return a fresh common config for deterministic behavior tests."""
    import yaml
    return yaml.safe_load((FIXTURES / 'mission_executor.yaml').read_text())


def load_fixture_task(name):
    """Use real task code/loader with explicit, test-owned YAML files."""
    from mission_executor.mission_loader import load_task
    return load_task(ROOT, name,
                     config=str(FIXTURES / (name + '.yaml')),
                     executor_config=str(FIXTURES / 'mission_executor.yaml'))


def takeoff_task():
    return load_fixture_task('takeoff_hover_land').task
