import ast
import re
import unittest
import xml.etree.ElementTree as ET
from support import ROOT, WORKSPACE
from uav_core.config import load_source


class StructureTests(unittest.TestCase):
    def test_source_schemas(self):
        for p in (ROOT/'config').glob('*/mock.yaml'):
            self.assertEqual(p,ROOT/'config/test/mock.yaml')
        for p in (ROOT/'config').glob('**/*.yaml'):
            if p.parent.name not in ('state_sources','test'):continue
            cfg=load_source(p)
            self.assertEqual(cfg['schema_version'],1)
            if p.stem!='mock':self.assertFalse(cfg['extrinsic']['calibrated'])
        self.assertFalse((ROOT/'config/sources').exists())
        self.assertFalse((ROOT/'config/state_sources/mock.yaml').exists())
        self.assertTrue((ROOT/'config/test/mock.yaml').is_file())
    def test_all_native_python_parses(self):
        ast.parse((ROOT/'setup.py').read_text(),filename=str(ROOT/'setup.py'))
        for d in ('support/uav_core','state_source_manager','state_adapter','flight_supervisor','px4_backend','commander'):
            for p in (ROOT/'src'/d).rglob('*.py'):ast.parse(p.read_text(),filename=str(p))
    def test_manifests_and_launch_xml(self):
        from catkin_pkg.package import parse_package
        pkg=parse_package(str(ROOT/'package.xml'))
        self.assertEqual(pkg.name,'uav_system')
        cmake=(ROOT/'CMakeLists.txt').read_text()
        exports={}
        for path in re.findall(r'\$\{CMAKE_CURRENT_SOURCE_DIR\}(/[^\s)]+\.py)',cmake):
            target=(ROOT/path.lstrip('/')).resolve()
            self.assertTrue(target.exists(),str(target))
            self.assertTrue(target.stat().st_mode&0o111)
            self.assertNotIn(target.name,exports)
            exports[target.name]=target
        for p in (ROOT/'launch').rglob('*.launch'):
            tree=ET.parse(p)
            for element in tree.iter():
                for value in element.attrib.values():
                    match=re.fullmatch(r'\$\(find uav_system\)(/[^$]+)',value)
                    if match:self.assertTrue((ROOT/match.group(1).lstrip('/')).exists(),(p,value))
            for node in tree.findall('.//node'):
                self.assertEqual(node.attrib['pkg'],'uav_system')
                self.assertIn(node.attrib['type'],exports)
        for folder in ('config','launch'):
            directory=ROOT/folder
            self.assertTrue(directory.is_dir());self.assertFalse(directory.is_symlink())
        self.assertFalse((ROOT/'src/px4ctrl').exists())
        self.assertFalse((ROOT/'src/estimator_adapter').exists())
    def test_root_layout_and_launch_references(self):
        for p in (ROOT/'config').glob('**/*.yaml'):
            if p.parent.name not in ('state_sources','test'):continue
            for item in load_source(p)['source']['launches']:
                root=ROOT if item['package']=='uav_system' else WORKSPACE/'src/open_vins'/item['package']
                self.assertEqual(len(list((root/'launch').glob('**/'+item['file']))),1,(p,item))
        cmake=(ROOT/'CMakeLists.txt').read_text()
        for folder in ('config','launch'):self.assertIn('/'+folder+'/',cmake)
        self.assertIn('/test/mock_state_source.py',cmake)
        self.assertTrue((ROOT/'test/test_state_adapter.py').exists())
        self.assertTrue((ROOT/'launch/commander.launch').exists())
    def test_catkin_discovers_one_native_package_and_openvins(self):
        from catkin_pkg.packages import find_packages
        packages=find_packages(str(WORKSPACE/'src'))
        native={p.name for path,p in packages.items() if not path.startswith('open_vins/')}
        self.assertEqual(native,{'uav_system'})
        self.assertIn('ov_msckf',{p.name for p in packages.values()})
        directory=WORKSPACE/'src/open_vins'
        self.assertTrue(directory.is_dir());self.assertFalse(directory.is_symlink())
        self.assertFalse((ROOT/'src/support/open_vins').exists())
        self.assertFalse((ROOT/'third_party/open_vins').exists())
        self.assertTrue((ROOT/'third_party/px4_autopilot').is_dir())
        self.assertEqual(set(packages), {'uav_system'} | {'open_vins/'+name for name in ('ov_core','ov_init','ov_msckf','ov_eval','ov_data')})
        self.assertFalse((WORKSPACE/'package.xml').exists())
        build=WORKSPACE/'scripts/build.sh'
        self.assertTrue(build.stat().st_mode&0o111)
        for folder in ('state_source_manager','state_adapter','flight_supervisor','px4_backend','commander'):
            for item in ('scripts','CMakeLists.txt','package.xml'):
                self.assertFalse((ROOT/'src'/folder/item).exists())
        for msg in ('SourceStatus','SystemStatus','EvStatus'):
            self.assertTrue((ROOT/'src/support/msg'/(msg+'.msg')).exists())
