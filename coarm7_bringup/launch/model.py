"""Build an independent hardware description without changing simulation files."""
from pathlib import Path
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

HARDWARE_COMPONENT = 'CoArm7Hardware'


def description(config_file, transport='fake', effective_limits_file=''):
    config = yaml.safe_load(Path(config_file).read_text())
    if not isinstance(config, dict) or config.get('transport') != transport:
        raise RuntimeError(f'Launch requires transport={transport}')
    if transport == 'sdk' and config.get('commissioning_confirmed') is not True:
        raise RuntimeError('Hardware launch requires an explicitly reviewed configuration with commissioning_confirmed: true')
    source = Path(get_package_share_directory('coarm7_description')) / 'urdf/coarm7.urdf.xacro'
    mappings = {'effective_limits_file': str(effective_limits_file), 'backend': transport, 'hardware_config_file': str(config_file),
                'gripper_joints': ' '.join(g['name'] for g in config['grippers'])}
    return {'robot_description': xacro.process_file(str(source), mappings=mappings).toxml()}
