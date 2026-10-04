import ast
import unittest
import xml.etree.ElementTree as ET
from test_support import ROOT
from uav_core.config import load_source


class StructureTests(unittest.TestCase):
    def test_source_schemas(self):
        for p in (ROOT/'config/sources').glob('*.yaml'):
            cfg=load_source(p)
            self.assertEqual(cfg['schema_version'],1)
            if p.stem!='mock':self.assertFalse(cfg['extrinsic']['calibrated'])
    def test_all_native_python_parses(self):
        for d in ('uav_core','state_source_manager','state_adapter','flight_supervisor','px4_backend','commander'):
            for p in (ROOT/'src'/('support/'+d if d=='uav_core' else d)).rglob('*.py'):ast.parse(p.read_text(),filename=str(p))
    def test_manifests_and_launch_xml(self):
        from catkin_pkg.package import parse_package
        packages={}
        for p in list((ROOT/'src').glob('*/package.xml'))+list((ROOT/'src/support').glob('*/package.xml')):
            if p.parent.name=='px4_autopilot':continue
            pkg=parse_package(str(p));packages[pkg.name]=p.parent
        for p in (ROOT/'launch').rglob('*.launch'):
            tree=ET.parse(p)
            for node in tree.findall('.//node'):
                if node.attrib['pkg'] in packages:
                    script=(ROOT/'test/mock_state_source.py' if node.attrib['type']=='mock_state_source.py'
                            else packages[node.attrib['pkg']]/'scripts'/node.attrib['type'])
                    self.assertTrue(script.exists(),str(script));self.assertTrue(script.stat().st_mode&0o111)
        self.assertTrue((ROOT/'third_party/px4_autopilot/CATKIN_IGNORE').exists())
        self.assertFalse((ROOT/'src/px4ctrl').exists())
        self.assertFalse((ROOT/'src/estimator_adapter').exists())
        for folder in ('config','launch'):
            link=ROOT/'src/support/uav_system'/folder
            self.assertTrue(link.is_symlink())
            self.assertEqual(link.resolve(),ROOT/folder)
        self.assertTrue((ROOT/'launch/commander.launch').exists())
        self.assertFalse((ROOT/'src/uav_commander').exists())
        self.assertFalse((ROOT/'docs/legacy').exists())

    def test_root_layout_and_launch_references(self):
        import yaml
        packages={'uav_system':ROOT/'src/support/uav_system',
                  'state_source_manager':ROOT/'src/state_source_manager'}
        for p in (ROOT/'config/sources').glob('*.yaml'):
            cfg=load_source(p)
            for item in cfg['source']['launches']:
                if item['package'] in packages:
                    matches=list((packages[item['package']]/'launch').glob('**/'+item['file']))
                    # pathlib can start glob at a linked directory in source space.
                    self.assertEqual(len(matches),1,(p,item))
        cmake=(ROOT/'src/support/uav_system/CMakeLists.txt').read_text()
        for folder in ('config','launch'):
            self.assertIn('/../../../'+folder+'/',cmake)
        source_cmake=(ROOT/'src/state_source_manager/CMakeLists.txt').read_text()
        self.assertIn('/../../test/mock_state_source.py',source_cmake)
        self.assertFalse((ROOT/'src/state_source_manager/scripts/mock_state_source.py').exists())
        self.assertTrue((ROOT/'test/test_state_adapter.py').exists())

    def test_catkin_discovers_nested_support_packages(self):
        from catkin_pkg.packages import find_packages
        packages=find_packages(str(ROOT/'src'))
        names={p.name for p in packages.values()}
        self.assertTrue({'commander','state_source_manager','state_adapter','flight_supervisor',
                         'px4_backend','uav_core','uav_msgs','uav_system','ov_msckf'}.issubset(names))
        self.assertNotIn('uav_commander',names)
