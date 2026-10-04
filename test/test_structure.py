import ast
import re
import unittest
import xml.etree.ElementTree as ET
from support import ROOT
from uav_core.config import load_source


class StructureTests(unittest.TestCase):
    def test_source_schemas(self):
        for p in (ROOT/'config/state_sources').glob('*.yaml'):
            cfg=load_source(p)
            self.assertEqual(cfg['schema_version'],1)
            if p.stem!='mock':self.assertFalse(cfg['extrinsic']['calibrated'])
        self.assertFalse((ROOT/'config/sources').exists())
    def test_all_native_python_parses(self):
        for d in ('support/uav_core','state_source_manager','state_adapter','flight_supervisor','px4_backend','commander','uav_system'):
            for p in (ROOT/'src'/d).rglob('*.py'):ast.parse(p.read_text(),filename=str(p))
    def test_manifests_and_launch_xml(self):
        from catkin_pkg.package import parse_package
        pkg=parse_package(str(ROOT/'src/uav_system/package.xml'))
        self.assertEqual(pkg.name,'uav_system')
        cmake=(ROOT/'src/uav_system/CMakeLists.txt').read_text()
        exports={}
        for path in re.findall(r'\$\{CMAKE_CURRENT_SOURCE_DIR\}(/[^\s)]+\.py)',cmake):
            target=(ROOT/'src/uav_system'/path.lstrip('/')).resolve()
            self.assertTrue(target.exists(),str(target))
            self.assertTrue(target.stat().st_mode&0o111)
            self.assertNotIn(target.name,exports)
            exports[target.name]=target
        for p in (ROOT/'launch').rglob('*.launch'):
            for node in ET.parse(p).findall('.//node'):
                self.assertEqual(node.attrib['pkg'],'uav_system')
                self.assertIn(node.attrib['type'],exports)
        for folder in ('config','launch'):
            link=ROOT/'src/uav_system'/folder
            self.assertTrue(link.is_symlink());self.assertEqual(link.resolve(),ROOT/folder)
        self.assertFalse((ROOT/'src/px4ctrl').exists())
        self.assertFalse((ROOT/'src/estimator_adapter').exists())
    def test_root_layout_and_launch_references(self):
        for p in (ROOT/'config/state_sources').glob('*.yaml'):
            for item in load_source(p)['source']['launches']:
                root=ROOT/'src/uav_system' if item['package']=='uav_system' else ROOT/'third_party/open_vins'/item['package']
                self.assertEqual(len(list((root/'launch').glob('**/'+item['file']))),1,(p,item))
        cmake=(ROOT/'src/uav_system/CMakeLists.txt').read_text()
        for folder in ('config','launch'):self.assertIn('/../../'+folder+'/',cmake)
        self.assertIn('/../../test/mock_state_source.py',cmake)
        self.assertTrue((ROOT/'test/test_state_adapter.py').exists())
        self.assertTrue((ROOT/'launch/commander.launch').exists())
    def test_catkin_discovers_one_native_package_and_openvins(self):
        from catkin_pkg.packages import find_packages
        packages=find_packages(str(ROOT/'src'))
        native={p.name for path,p in packages.items() if not path.startswith('support/open_vins/')}
        self.assertEqual(native,{'uav_system'})
        self.assertIn('ov_msckf',{p.name for p in packages.values()})
        link=ROOT/'src/support/open_vins'
        self.assertTrue(link.is_symlink());self.assertEqual(link.resolve(),ROOT/'third_party/open_vins')
        self.assertFalse((ROOT/'src/open_vins').exists())
        for folder in ('state_source_manager','state_adapter','flight_supervisor','px4_backend','commander'):
            for item in ('scripts','CMakeLists.txt','package.xml'):
                self.assertFalse((ROOT/'src'/folder/item).exists())
        for msg in ('SourceStatus','SystemStatus','EvStatus'):
            self.assertTrue((ROOT/'src/support/msg'/(msg+'.msg')).exists())
