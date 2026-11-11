"""Confirm hardware and planning use the same URDF arm position bounds."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml


def load_report(path, config_file, component='CoArm7Hardware'):
    report = yaml.safe_load(Path(path).read_text())
    config = yaml.safe_load(Path(config_file).read_text())
    if (not isinstance(report, dict) or report.get('schema') != 1
            or report.get('coordinate_system') != 'ros_joint_radians'
            or report.get('component') != component
            or report.get('config_file') != str(Path(config_file).resolve())):
        raise ValueError('Effective limit snapshot does not belong to this hardware session')
    configured = {axis['name']: axis for axis in config['axes']}
    axes = report.get('axes')
    if len(configured) != 14 or not isinstance(axes, dict) or set(axes) != set(configured):
        raise ValueError('Effective limits must contain exactly the configured 14 arm joints')
    for name, bounds in axes.items():
        if not isinstance(bounds, dict): raise ValueError('Invalid bounds for ' + name)
        lo, hi = bounds.get('lower'), bounds.get('upper')
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in [lo, hi]) or lo >= hi:
            raise ValueError('Invalid finite ordered bounds for ' + name)
    return report


def apply_limits(robot, report):
    root = ET.fromstring(robot['robot_description'])
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    applied = {}
    for name, bounds in report['axes'].items():
        if name not in joints: raise ValueError('Missing model joint: ' + name)
        joint = joints[name]; limit = joint.find('limit')
        if joint.get('type') != 'revolute' or limit is None: raise ValueError('Expected bounded arm joint: ' + name)
        lo, hi = float(limit.get('lower')), float(limit.get('upper'))
        if not all(math.isfinite(v) for v in [lo, hi]) or lo >= hi: raise ValueError('Invalid URDF limits: ' + name)
        if not (math.isclose(lo, bounds['lower'], rel_tol=0, abs_tol=1e-12)
                and math.isclose(hi, bounds['upper'], rel_tol=0, abs_tol=1e-12)):
            raise ValueError('Hardware position limits differ from URDF: ' + name)
        applied[name] = {'lower': lo, 'upper': hi}
    return dict(robot), applied


def model_position_parameters(robot):
    root = ET.fromstring(robot['robot_description'])
    result = {}
    for joint in root.findall('joint'):
        name = joint.get('name', '')
        if name not in {f'{side}_joint{i}' for side in ['L', 'R'] for i in range(1, 8)}: continue
        limit = joint.find('limit')
        if limit is None: raise ValueError('Missing arm position limit: ' + name)
        lo, hi = float(limit.get('lower')), float(limit.get('upper'))
        if not all(math.isfinite(v) for v in [lo, hi]) or lo >= hi: raise ValueError('Invalid model bounds: ' + name)
        result[name] = {'has_position_limits': True, 'min_position': lo, 'max_position': hi}
    if len(result) != 14: raise ValueError('Robot model must contain 14 bounded arm joints')
    return result
