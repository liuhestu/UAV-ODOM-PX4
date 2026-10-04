import ast
import unittest
import xml.etree.ElementTree as ET
from test_support import ROOT
from uav_core.config import load_source


class StructureTests(unittest.TestCase):
    def test_source_schemas(self):
        for p in (ROOT/'src/uav_system/config/sources').glob('*.yaml'):
            cfg=load_source(p)
            self.assertEqual(cfg['schema_version'],1)
            if p.stem!='mock':self.assertFalse(cfg['extrinsic']['calibrated'])
    def test_all_native_python_parses(self):
        for d in ('uav_core','state_source_manager','state_adapter','flight_supervisor','px4_backend','uav_commander'):
            for p in (ROOT/'src'/d).rglob('*.py'):ast.parse(p.read_text(),filename=str(p))
    def test_manifests_and_launch_xml(self):
        from catkin_pkg.package import parse_package
        packages={}
        for p in (ROOT/'src').glob('*/package.xml'):
            if p.parent.name=='px4_autopilot':continue
            pkg=parse_package(str(p));packages[pkg.name]=p.parent
        for d in ('uav_system','state_source_manager'):
            for p in (ROOT/'src'/d).rglob('*.launch'):
                tree=ET.parse(p)
                for node in tree.findall('.//node'):
                    if node.attrib['pkg'] in packages:
                        script=packages[node.attrib['pkg']]/'scripts'/node.attrib['type']
                        self.assertTrue(script.exists(),str(script));self.assertTrue(script.stat().st_mode&0o111)
        self.assertTrue((ROOT/'src/px4_autopilot/CATKIN_IGNORE').exists())
        self.assertFalse((ROOT/'src/px4ctrl').exists())
        self.assertFalse((ROOT/'src/estimator_adapter').exists())
