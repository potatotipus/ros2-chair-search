#!/usr/bin/env python3
"""Prepare Linux-local assets using installed ROS configs and cached chair."""
import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml

from room_geometry import SPAWN, write_map, write_world, write_simple_chair


def find_chair(root):
    candidates = []
    if root.exists():
        for config in root.rglob('model.config'):
            try:
                name = (ET.parse(config).findtext('name') or '').strip().casefold()
                if name == 'office chair' and (config.parent / 'model.sdf').is_file():
                    candidates.append(config.parent)
            except (ET.ParseError, OSError):
                continue
    if not candidates:
        raise RuntimeError('Cached office chair not found. Use --chair /path/to/office-chair-folder.')
    return max(candidates, key=lambda p: (p / 'model.sdf').stat().st_mtime)


def prepare(output, chair, localization_source, navigation_source, description_source):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    source = Path(description_source).read_text(encoding='utf-8')
    import re
    if re.search(r'<render_engine>\s*ogre\s*</render_engine>', source):
        raise RuntimeError('Sensors still use ogre. Apply the confirmed ogre2 LiDAR fix first.')
    if not re.search(r'<render_engine>\s*ogre2\s*</render_engine>', source):
        raise RuntimeError('Cannot confirm ogre2 Sensors setting; inspect create3.urdf.xacro.')
    chair = Path(chair).resolve()
    if not (chair / 'model.sdf').is_file():
        raise RuntimeError(f'Missing chair model.sdf in {chair}')
    write_world(output / 'room.sdf', chair)
    write_map(output)
    localization = yaml.safe_load(Path(localization_source).read_text(encoding='utf-8'))
    amcl = localization['amcl']['ros__parameters']
    amcl.update({
        'use_sim_time': True, 'set_initial_pose': True,
        'initial_pose': {'x': SPAWN[0], 'y': SPAWN[1], 'z': 0.0, 'yaw': SPAWN[2]},
        'max_particles': 1000, 'min_particles': 300,
        'laser_max_range': 12.0, 'update_min_a': 0.1, 'update_min_d': 0.1,
    })
    (output / 'localization.yaml').write_text(yaml.safe_dump(localization, sort_keys=False), encoding='utf-8')
    navigation = yaml.safe_load(Path(navigation_source).read_text(encoding='utf-8'))
    control = navigation['controller_server']['ros__parameters']
    control['controller_frequency'] = 10.0
    follow = control.get('FollowPath', {})
    if 'mppi' in str(follow.get('plugin', '')).casefold():
        # Match MPPI model_dt to the 10 Hz controller; keep stock horizon/critics.
        follow.update({'model_dt': 0.1, 'time_steps': 30, 'batch_size': 1000,
                       'vx_max': 0.25, 'visualize': False})
    smoother = navigation.get('velocity_smoother', {}).get('ros__parameters', {})
    if smoother.get('max_velocity'):
        smoother['max_velocity'][0] = 0.25
    (output / 'nav2.yaml').write_text(yaml.safe_dump(navigation, sort_keys=False), encoding='utf-8')
    print(f'Ready: {output}')
    print(f'Chair: {chair}')
    print('Start position: x=-2.5, y=-1.7, yaw=0. Initial localization is automatic.')
    print('Next: bash demo.sh sim')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--chair', type=Path)
    args = parser.parse_args()
    from ament_index_python.packages import get_package_share_directory
    nav = Path(get_package_share_directory('turtlebot4_navigation'))
    description = Path(get_package_share_directory('irobot_create_description'))
    runtime = Path(__file__).resolve().parent / 'runtime'
    chair = args.chair or write_simple_chair(runtime / 'simple_chair')
    prepare(runtime, chair,
            nav / 'config' / 'localization.yaml', nav / 'config' / 'nav2.yaml',
            description / 'urdf' / 'create3.urdf.xacro')


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, KeyError, yaml.YAMLError) as error:
        print(f'PREPARE ERROR: {error}', file=sys.stderr)
        sys.exit(1)
