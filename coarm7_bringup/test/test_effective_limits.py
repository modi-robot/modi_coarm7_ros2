from pathlib import Path
import copy
import math
import runpy
import xml.etree.ElementTree as ET
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
limits = runpy.run_path(str(ROOT / 'coarm7_bringup/launch/effective_limits.py'))


@pytest.fixture
def example(tmp_path):
    config = yaml.safe_load((ROOT / 'coarm7_hardware/config/offline.yaml').read_text())
    path = tmp_path / 'reviewed.yaml'; path.write_text(yaml.safe_dump(config))
    report = {'schema': 1, 'component': 'CoArm7Hardware', 'coordinate_system': 'ros_joint_radians',
              'config_file': str(path), 'axes': {a['name']: {'lower': a['lower'], 'upper': a['upper']} for a in config['axes']}}
    root = ET.Element('robot', name='test')
    for axis in config['axes']:
        joint = ET.SubElement(root, 'joint', name=axis['name'], type='revolute')
        ET.SubElement(joint, 'limit', lower=str(axis['lower']), upper=str(axis['upper']), velocity='0.5', effort='4.9')
        ET.SubElement(joint, 'origin', xyz='1 2 3', rpy='0 0 1.57')
    g = ET.SubElement(root, 'joint', name='gripper', type='revolute')
    ET.SubElement(g, 'limit', lower='0', upper='.864', velocity='.12')
    return path, report, {'robot_description': ET.tostring(root, encoding='unicode')}


def test_urdf_bounds_match_hardware_without_touching_geometry_or_gripper(example, tmp_path):
    config, report, robot = example
    p = tmp_path / 'report.yaml'; p.write_text(yaml.safe_dump(report))
    loaded = limits['load_report'](p, config)
    result, applied = limits['apply_limits'](robot, loaded)
    planning = limits['model_position_parameters'](result)
    assert planning['L_joint5']['max_position'] == report['axes']['L_joint5']['upper']
    assert planning['L_joint5']['max_position'] == 3.071779
    assert planning['L_joint5']['has_position_limits'] is True
    before, after = ET.fromstring(robot['robot_description']), ET.fromstring(result['robot_description'])
    assert ET.tostring(before.find("joint[@name='gripper']")) == ET.tostring(after.find("joint[@name='gripper']"))
    for joint in before.findall('joint'):
        other = after.find("joint[@name='%s']" % joint.get('name'))
        for child in joint:
            if child.tag != 'limit': assert ET.tostring(child) == ET.tostring(other.find(child.tag))
    assert len(applied) == 14


@pytest.mark.parametrize('bad', ['missing', 'unknown', 'nan', 'reversed', 'wrong_session', 'wrong_units'])
def test_invalid_snapshot_never_falls_back_to_wider_model(example, tmp_path, bad):
    config, report, _ = example
    if bad == 'missing': del report['axes']['L_joint5']
    if bad == 'unknown': report['axes']['other'] = report['axes'].pop('L_joint5')
    if bad == 'nan': report['axes']['L_joint5']['lower'] = float('nan')
    if bad == 'reversed': report['axes']['L_joint5'] = {'lower': 1, 'upper': 0}
    if bad == 'wrong_session': report['config_file'] = '/different/config.yaml'
    if bad == 'wrong_units': report['coordinate_system'] = 'degrees'
    p = tmp_path / 'report.yaml'; p.write_text(yaml.safe_dump(report))
    with pytest.raises(ValueError): limits['load_report'](p, config)


@pytest.mark.parametrize('bounds', [{'lower': -100, 'upper': 100}, {'lower': -.1, 'upper': .1}])
def test_mismatched_hardware_report_is_rejected_not_intersected(example, bounds):
    _, report, robot = example
    report['axes']['L_joint1'] = bounds
    with pytest.raises(ValueError, match='differ from URDF'): limits['apply_limits'](robot, report)


def test_yaml_position_values_do_not_narrow_urdf(example, tmp_path):
    config, report, robot = example
    data = yaml.safe_load(config.read_text())
    data['axes'][0]['lower'] = -.1; data['axes'][0]['upper'] = .1
    config.write_text(yaml.safe_dump(data))
    p = tmp_path / 'report.yaml'; p.write_text(yaml.safe_dump(report))
    result, _ = limits['apply_limits'](robot, limits['load_report'](p, config))
    assert result == robot
