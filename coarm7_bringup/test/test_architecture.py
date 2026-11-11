from pathlib import Path
import runpy
import xml.etree.ElementTree as ET
import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parents[2]

def signature(element):
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or '').strip(), tuple(signature(e) for e in element))

@pytest.mark.parametrize('transport', ['mock', 'fake', 'sdk'])
def test_generated_model_matches_previous_interface_contract(transport):
    source = Path(get_package_share_directory('coarm7_description')) / 'urdf/coarm7.urdf.xacro'
    baseline = ET.fromstring(xacro.process_file(str(source)).toxml())
    path = ROOT / 'coarm7_hardware/config' / ('hardware_pending.yaml' if transport == 'sdk' else 'offline.yaml')
    config = yaml.safe_load(path.read_text())
    actual = ET.fromstring(xacro.process_file(str(source), mappings={
        'backend': transport, 'hardware_config_file': str(path),
        'gripper_joints': ' '.join(g['name'] for g in config['grippers'])}).toxml())
    if transport != 'mock':
        control = baseline.find('ros2_control')
        control.set('name', 'CoArm7Hardware' if transport == 'sdk' else 'CoArm7HardwareOffline')
        hardware = control.find('hardware'); hardware.clear()
        ET.SubElement(hardware, 'plugin').text = 'coarm7_hardware/CoArm7System'
        ET.SubElement(hardware, 'param', {'name': 'config_file'}).text = str(path)
        masters = {g['name'] for g in config['grippers']}
        for joint in control.findall('joint'):
            if joint.get('name') in masters:
                for key in ('goal_id', 'stop_id'): ET.SubElement(joint, 'command_interface', {'name': key})
                for key in ('feedback_stamp', 'stop_ack'): ET.SubElement(joint, 'state_interface', {'name': key})
    assert signature(actual) == signature(baseline)


def test_hardware_review_and_transport_guards_remain(tmp_path):
    module = runpy.run_path(str(ROOT / 'coarm7_bringup/launch/model.py'))
    pending = ROOT / 'coarm7_hardware/config/hardware_pending.yaml'
    with pytest.raises(RuntimeError, match='reviewed'): module['description'](pending, 'sdk')
    with pytest.raises(RuntimeError, match='transport'): module['description'](pending, 'fake')


def test_dependencies_are_acyclic_and_no_sdk_for_mock_or_controller():
    packages = {}
    for path in ROOT.glob('*/package.xml'):
        doc = ET.parse(path).getroot()
        packages[doc.findtext('name')] = {e.text for e in doc if 'depend' in e.tag}
    def walk(name, stack=()):
        assert name not in stack, stack + (name,)
        result = set()
        for dep in packages.get(name, set()): result.add(dep); result.update(walk(dep, stack + (name,)))
        return result
    for name in packages: walk(name)
    for name in ['coarm7_bringup','coarm7_controllers']:
        assert not {'canevo_sdk_x86','coarm7_hardware','coarm7_diagnostics'} & walk(name)
    assert not {'coarm7_bringup','coarm7_moveit_config','coarm7_planning','coarm7_controllers'} & walk('coarm7_hardware')
    assert not any('gazebo' in dep or 'ros_gz' in dep or 'gz_ros' in dep for deps in packages.values() for dep in deps)


def test_no_private_deployment_packaging_in_planner():
    cmake = (ROOT / 'coarm7_planning/CMakeLists.txt').read_text()
    assert 'BundlePilzRuntime' not in cmake and 'CMAKE_CROSSCOMPILING' not in cmake
