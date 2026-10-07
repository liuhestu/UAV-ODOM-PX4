"""Direct source node declarations are validated before roslaunch starts them."""
import copy
import tempfile
from pathlib import Path
import unittest
import yaml
from support import ROOT
from uav_core.source_config import load_source


class DirectSourceConfigTests(unittest.TestCase):
    def setUp(self):
        self.cfg = yaml.safe_load((ROOT / 'config/state_sources/mock.yaml').read_text())

    def load(self, cfg):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'source.yaml'
            path.write_text(yaml.safe_dump(cfg))
            return load_source(path)

    def test_direct_node_and_existing_launch_sources(self):
        loaded = self.load(self.cfg)
        self.assertEqual(loaded['source']['launches'], [])
        self.assertEqual(loaded['source']['nodes'][0]['executable'], 'mock_state_source.py')
        for name in ('openvins', 'nokov'):
            load_source(ROOT / 'config/state_sources' / (name + '.yaml'))

    def test_invalid_node_identity_and_duplicates(self):
        for field, value in (('package', '../uav_system'), ('name', '/mock_state_source'),
                             ('executable', '../mock_state_source.py'), ('executable', None)):
            cfg = copy.deepcopy(self.cfg)
            cfg['source']['nodes'][0][field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                self.load(cfg)
        cfg = copy.deepcopy(self.cfg)
        cfg['source']['nodes'].append(copy.deepcopy(cfg['source']['nodes'][0]))
        with self.assertRaises(ValueError):
            self.load(cfg)

    def test_parameter_types_and_invalid_node_container(self):
        for params in (['static'], {'mode': ['static']}, {'rate': float('nan')}, {'bad/name': 1}):
            cfg = copy.deepcopy(self.cfg)
            cfg['source']['nodes'][0]['params'] = params
            with self.subTest(params=params), self.assertRaises(ValueError):
                self.load(cfg)
        cfg = copy.deepcopy(self.cfg)
        cfg['source']['nodes'] = {}
        with self.assertRaises(ValueError):
            self.load(cfg)
