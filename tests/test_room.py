"""Offline validation of the generated room, prior map, and ROS launch wiring."""
import ast
from collections import deque
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import room_geometry as geo
import prepare_demo


class RoomTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.chair = self.root / 'office chair' / '3'
        self.chair.mkdir(parents=True)
        (self.chair / 'model.sdf').write_text('<sdf version="1.9"><model name="chair"/></sdf>')
        (self.chair / 'model.config').write_text('<model><name>Office Chair</name></model>')

    def test_world_and_map_share_exact_obstacle_geometry(self):
        world_file = self.root / 'room.sdf'
        geo.write_world(world_file, self.chair)
        world = ET.parse(world_file).find('world')
        self.assertEqual(world.attrib['name'], 'warehouse')
        for name, x, y, sx, sy, sz, color in geo.BOXES:
            model = world.find(f"model[@name='{name}']")
            self.assertIsNotNone(model)
            self.assertEqual(list(map(float, model.findtext('pose').split()))[:2], [x, y])
            self.assertEqual(list(map(float, model.findtext('link/collision/geometry/box/size').split())), [sx, sy, sz])
        pixels = geo.write_map(self.root)
        self.assertEqual(len(pixels), geo.WIDTH * geo.HEIGHT)
        for row in range(geo.HEIGHT):
            for col in range(geo.WIDTH):
                x = geo.ORIGIN[0] + (col + 0.5) * geo.RESOLUTION
                y = geo.ORIGIN[1] + (geo.HEIGHT - row - 0.5) * geo.RESOLUTION
                self.assertEqual(pixels[row * geo.WIDTH + col] == 0, geo.occupied(x, y))
        metadata = yaml.safe_load((self.root / 'room.yaml').read_text())
        self.assertGreater((255 - 205) / 255, metadata['free_thresh'])
        self.assertLess((255 - 205) / 255, metadata['occupied_thresh'])
        self.assertEqual(world.findtext('include/name'), 'ai_chair')
        self.assertEqual(world.findtext('include/static'), 'true')
        chair_pose = list(map(float, world.findtext('include/pose').split()))
        self.assertAlmostEqual(chair_pose[5], math.pi)
        self.assertEqual(world.findtext('include/uri'), str(self.chair.resolve()))
        self.assertNotIn('%20', world.findtext('include/uri'))
        self.assertTrue(Path(world.findtext('include/uri'), 'model.sdf').is_file())

    def test_target_hidden_initially_and_reachable_with_clearance(self):
        # Chair should be hidden by the divider before the robot moves.
        sx, sy, _ = geo.SPAWN
        tx, ty, _ = geo.CHAIR
        self.assertTrue(any(geo.occupied(sx + (tx-sx)*i/1000, sy + (ty-sy)*i/1000)
                            for i in range(1001)))
        # Search on a 10 cm grid with 45 cm clearance around fixed boxes.
        # This exceeds the robot body radius and leaves room for inflation.
        step, margin = 0.1, 0.45
        def blocked(cell):
            x, y = cell[0]*step, cell[1]*step
            if abs(x) > 3.5 or abs(y) > 2.5:
                return True
            return any(abs(x-cx) <= bx/2+margin and abs(y-cy) <= by/2+margin
                       for _, cx, cy, bx, by, _, _ in geo.BOXES)
        start = (round((sx-0.5)/step), round(sy/step))
        goal = tuple(round(v/step) for v in geo.OBSERVATION)
        self.assertFalse(blocked(start))
        self.assertFalse(blocked(goal))
        queue, seen = deque([start]), {start}
        while queue:
            cell = queue.popleft()
            if cell == goal:
                break
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt = (cell[0]+dx, cell[1]+dy)
                if nxt not in seen and not blocked(nxt):
                    seen.add(nxt)
                    queue.append(nxt)
        self.assertIn(goal, seen)
        gx, gy = geo.OBSERVATION
        self.assertFalse(any(geo.occupied(gx+(tx-gx)*i/100, gy+(ty-gy)*i/100) for i in range(101)))

    def test_simple_chair_is_complete_static_and_self_contained(self):
        directory = geo.write_simple_chair(self.root/'simple_chair')
        root = ET.parse(directory/'model.sdf')
        self.assertEqual(root.findtext('model/static'), 'true')
        visuals = root.findall('model/link/visual')
        collisions = root.findall('model/link/collision')
        self.assertEqual(len(visuals), 8)
        self.assertEqual(len(collisions), 8)
        self.assertEqual(len([v for v in visuals if v.attrib['name'].startswith('leg_')]), 4)
        self.assertFalse(root.findall('.//mesh'))
        self.assertFalse(root.findall('.//include'))
        self.assertEqual(ET.parse(directory/'model.config').findtext('sdf'), 'model.sdf')
        for visual in visuals:
            values = list(map(float, visual.findtext('geometry/box/size').split()))
            self.assertTrue(all(v > 0 for v in values))

    def make_config_sources(self):
        localization = self.root / 'source_localization.yaml'
        localization.write_text(yaml.safe_dump({'amcl': {'ros__parameters': {'base_frame_id': 'base_link'}}}))
        nav = self.root / 'source_nav.yaml'
        nav.write_text(yaml.safe_dump({
            'controller_server': {'ros__parameters': {
                'controller_frequency': 20.0,
                'FollowPath': {'plugin': 'nav2_mppi_controller::MPPIController', 'vx_max': 0.5}}},
            'velocity_smoother': {'ros__parameters': {'max_velocity': [0.5, 0.0, 1.9]}}}))
        description = self.root / 'create3.xacro'
        description.write_text('<render_engine>ogre2</render_engine>')
        return localization, nav, description

    def test_preparation_auto_pose_controller_and_old_engine_guard(self):
        localization, nav, description = self.make_config_sources()
        output = self.root / 'runtime'
        prepare_demo.prepare(output, self.chair, localization, nav, description)
        amcl = yaml.safe_load((output/'localization.yaml').read_text())['amcl']['ros__parameters']
        self.assertTrue(amcl['set_initial_pose'])
        self.assertEqual(amcl['initial_pose'], {'x': -2.5, 'y': -1.7, 'z': 0.0, 'yaw': 0.0})
        control = yaml.safe_load((output/'nav2.yaml').read_text())['controller_server']['ros__parameters']
        self.assertEqual(control['controller_frequency'] * control['FollowPath']['model_dt'], 1)
        self.assertEqual(control['FollowPath']['vx_max'], 0.25)
        self.assertEqual(prepare_demo.find_chair(self.root), self.chair)
        description.write_text('<render_engine>ogre</render_engine>')
        with self.assertRaisesRegex(RuntimeError, 'ogre2'):
            prepare_demo.prepare(output, self.chair, localization, nav, description)

    def test_all_python_yaml_and_gui_syntax(self):
        for path in ROOT.glob('*.py'):
            ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
        config = yaml.safe_load((ROOT/'room.rviz').read_text(encoding='utf-8'))
        display = config['Visualization Manager']['Displays']
        self.assertEqual(display[0]['Alpha'], 1)
        self.assertEqual(display[0]['Topic']['Value'], '/map')
        self.assertEqual(display[1]['Color Transformer'], 'FlatColor')
        xml = (ROOT/'room_gui.config').read_text(encoding='utf-8').split('?>', 1)[1]
        gui = ET.fromstring('<config>' + xml + '</config>')
        self.assertEqual(gui.find("plugin[@filename='MinimalScene']/engine").text, 'ogre2')
        video = gui.find("plugin[@filename='VideoRecorder']/record_video")
        self.assertEqual(video.findtext('lockstep'), 'true')

    def test_sim_launch_includes_one_robot_and_no_warehouse_geometry(self):
        captured = []
        def action(kind):
            return lambda *args, **kw: captured.append((kind, args, kw)) or types.SimpleNamespace(kind=kind, kwargs=kw)
        modules = {}
        def module(name, **attrs):
            m = types.ModuleType(name)
            m.__dict__.update(attrs)
            modules[name] = m
        module('ament_index_python')
        module('ament_index_python.packages', get_package_share_directory=lambda p: str(self.root/'ros'/p))
        module('launch', LaunchDescription=lambda actions: actions)
        module('launch.actions', **{n: action(n) for n in ('ExecuteProcess', 'IncludeLaunchDescription',
               'RegisterEventHandler', 'SetEnvironmentVariable', 'EmitEvent')})
        module('launch.event_handlers', OnProcessExit=action('OnProcessExit'))
        module('launch.events', Shutdown=action('Shutdown'))
        module('launch.launch_description_sources', PythonLaunchDescriptionSource=action('Source'))
        module('launch_ros')
        module('launch_ros.actions', Node=action('Node'))
        runtime = self.root/'runtime'
        runtime.mkdir()
        (runtime/'room.sdf').write_text('<sdf/>')
        target = self.root/'room_demo.launch.py'
        target.write_text((ROOT/'room_demo.launch.py').read_text(encoding='utf-8'))
        spec = importlib.util.spec_from_file_location('room_launch_test', target)
        loaded = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(loaded)
            actions = loaded.generate_launch_description()
        self.assertEqual(len(actions), 5)
        cmd = next(kw['cmd'] for kind, args, kw in captured if kind == 'ExecuteProcess')
        self.assertIn(str(runtime/'room.sdf'), cmd)
        self.assertIn('--gui-config', cmd)
        robot = next(kw for kind, args, kw in captured if kind == 'IncludeLaunchDescription')
        params = dict(robot['launch_arguments'])
        self.assertEqual(float(params['x']), geo.SPAWN[0])
        self.assertEqual(params['model'], 'standard')
        self.assertEqual(params['nav2'], 'false')


if __name__ == '__main__':
    unittest.main(verbosity=2)
