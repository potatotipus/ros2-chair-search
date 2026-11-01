"""Single source of truth for the small demonstration room and its prior map."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

SPAWN = (-2.5, -1.7, 0.0)
OBSERVATION = (0.4, 1.2)
CHAIR = (3.1, 1.2, math.pi)
RESOLUTION = 0.05
ORIGIN = (-4.5, -3.5)
WIDTH, HEIGHT = 180, 140
# name, center x/y, size x/y/z, diffuse RGBA
BOXES = [
    ('west_wall', -4.0, 0.0, 0.15, 6.15, 1.2, '0.75 0.80 0.85 1'),
    ('east_wall', 4.0, 0.0, 0.15, 6.15, 1.2, '0.75 0.80 0.85 1'),
    ('north_wall', 0.0, 3.0, 8.15, 0.15, 1.2, '0.75 0.80 0.85 1'),
    ('south_wall', 0.0, -3.0, 8.15, 0.15, 1.2, '0.75 0.80 0.85 1'),
    ('divider', 0.0, -0.7, 0.25, 2.4, 1.1, '0.15 0.40 0.65 1'),
    ('box', -0.8, 1.65, 0.65, 0.65, 0.65, '0.85 0.60 0.15 1'),
]


def text(parent, tag, value):
    ET.SubElement(parent, tag).text = str(value)


def write_simple_chair(directory):
    """Create a single static model: four legs, seat, two posts and backrest.

    Local +X is the sitting/front direction. At yaw pi the front faces the
    observation point to the west. All dimensions are in meters.
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    root = ET.Element('sdf', version='1.9')
    model = ET.SubElement(root, 'model', name='simple_chair')
    text(model, 'static', 'true')
    link = ET.SubElement(model, 'link', name='chair_link')
    parts = [('seat', 0, 0, 0.46, 0.50, 0.50, 0.07)]
    for i, x in enumerate((-0.205, 0.205)):
        for j, y in enumerate((-0.205, 0.205)):
            parts.append((f'leg_{i}_{j}', x, y, 0.2125, 0.055, 0.055, 0.425))
    for i, y in enumerate((-0.215, 0.215)):
        parts.append((f'back_post_{i}', -0.215, y, 0.70, 0.055, 0.055, 0.55))
    parts.append(('backrest', -0.215, 0, 0.86, 0.055, 0.49, 0.23))
    for name, x, y, z, sx, sy, sz in parts:
        for kind in ('collision', 'visual'):
            element = ET.SubElement(link, kind, name=f'{name}_{kind}')
            text(element, 'pose', f'{x} {y} {z} 0 0 0')
            text(ET.SubElement(ET.SubElement(element, 'geometry'), 'box'), 'size', f'{sx} {sy} {sz}')
            if kind == 'visual':
                material = ET.SubElement(element, 'material')
                color = '0.66 0.32 0.10 1' if name in ('seat', 'backrest') else '0.40 0.18 0.06 1'
                text(material, 'ambient', color)
                text(material, 'diffuse', color)
                text(element, 'cast_shadows', 'false')
    ET.indent(root)
    ET.ElementTree(root).write(directory / 'model.sdf', encoding='utf-8', xml_declaration=True)
    config = ET.Element('model')
    text(config, 'name', 'Simple Four Leg Chair')
    text(config, 'version', '1.0')
    sdf = ET.SubElement(config, 'sdf', version='1.9')
    sdf.text = 'model.sdf'
    text(config, 'description', 'Original static four-leg wooden chair for the task demonstration.')
    ET.indent(config)
    ET.ElementTree(config).write(directory / 'model.config', encoding='utf-8', xml_declaration=True)
    return directory


def static_box(world, name, x, y, sx, sy, sz, color, z=None):
    model = ET.SubElement(world, 'model', name=name)
    text(model, 'static', 'true')
    text(model, 'pose', f'{x} {y} {sz/2 if z is None else z} 0 0 0')
    link = ET.SubElement(model, 'link', name='link')
    for kind in ('collision', 'visual'):
        element = ET.SubElement(link, kind, name=kind)
        geometry = ET.SubElement(element, 'geometry')
        text(ET.SubElement(geometry, 'box'), 'size', f'{sx} {sy} {sz}')
        if kind == 'visual':
            material = ET.SubElement(element, 'material')
            text(material, 'ambient', color)
            text(material, 'diffuse', color)
            text(element, 'cast_shadows', 'false')


def marker(world, name, x, y, color):
    model = ET.SubElement(world, 'model', name=name)
    text(model, 'static', 'true')
    text(model, 'pose', f'{x} {y} 0.003 0 0 0')
    visual = ET.SubElement(ET.SubElement(model, 'link', name='link'), 'visual', name='marker')
    cylinder = ET.SubElement(ET.SubElement(visual, 'geometry'), 'cylinder')
    text(cylinder, 'radius', 0.28)
    text(cylinder, 'length', 0.004)
    material = ET.SubElement(visual, 'material')
    text(material, 'ambient', color)
    text(material, 'diffuse', color)
    text(visual, 'cast_shadows', 'false')


def write_world(path, chair_model):
    root = ET.Element('sdf', version='1.9')
    # Installed TurtleBot bridges default to /world/warehouse; retain that
    # transport identifier while replacing all warehouse geometry.
    world = ET.SubElement(root, 'world', name='warehouse')
    physics = ET.SubElement(world, 'physics', name='demo_physics', type='ignored')
    text(physics, 'max_step_size', 0.003)
    text(physics, 'real_time_factor', 1.0)
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                           ('scene-broadcaster', 'SceneBroadcaster'), ('contact', 'Contact')]:
        plugin = ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system',
                               name=f'gz::sim::systems::{name}')
        if name == 'SceneBroadcaster':
            text(plugin, 'state_hertz', 25)
    # Robot URDF supplies the ogre2 Sensors system, as in the installed simulator.
    scene = ET.SubElement(world, 'scene')
    text(scene, 'ambient', '0.65 0.65 0.65 1')
    text(scene, 'background', '0.85 0.90 0.95 1')
    text(scene, 'shadows', 'false')
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    text(light, 'pose', '0 0 8 0 0 0')
    text(light, 'cast_shadows', 'false')
    text(light, 'diffuse', '0.8 0.8 0.8 1')
    text(light, 'specular', '0.1 0.1 0.1 1')
    text(light, 'direction', '-0.3 -0.4 -1')
    attenuation = ET.SubElement(light, 'attenuation')
    text(attenuation, 'range', 100)
    text(attenuation, 'constant', 1)
    text(attenuation, 'linear', 0)
    text(attenuation, 'quadratic', 0)
    static_box(world, 'floor', 0, 0, 8.3, 6.3, 0.05,
               '0.82 0.80 0.73 1', z=-0.025)
    for box in BOXES:
        static_box(world, *box)
    marker(world, 'start_marker', *SPAWN[:2], '0.15 0.55 0.90 1')
    include = ET.SubElement(world, 'include')
    # libsdformat accepts an absolute filesystem path. Do not URL-encode spaces:
    # Fuel's cached directory can literally be named "office chair".
    text(include, 'uri', str(Path(chair_model).resolve()))
    text(include, 'name', 'ai_chair')
    text(include, 'pose', f'{CHAIR[0]} {CHAIR[1]} 0.02 0 0 {CHAIR[2]}')
    text(include, 'static', 'true')
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def occupied(x, y):
    return any(abs(x - cx) <= sx/2 and abs(y - cy) <= sy/2
               for _, cx, cy, sx, sy, _, _ in BOXES)


def write_map(directory):
    """Rasterize the exact static box geometry, not a hand-aligned SLAM map.

    The target chair is intentionally absent from the prior map. Nav2's scan
    costmaps still see it as an obstacle; AI is responsible for its recognition.
    """
    pixels = bytearray()
    for row in range(HEIGHT):
        y = ORIGIN[1] + (HEIGHT - row - 0.5) * RESOLUTION
        for col in range(WIDTH):
            x = ORIGIN[0] + (col + 0.5) * RESOLUTION
            inside = abs(x) <= 4.075 and abs(y) <= 3.075
            pixels.append(0 if occupied(x, y) else (254 if inside else 205))
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory / 'room.pgm').write_bytes(f'P5\n{WIDTH} {HEIGHT}\n255\n'.encode() + pixels)
    (directory / 'room.yaml').write_text(
        'image: room.pgm\nmode: trinary\nresolution: 0.05\n'
        'origin: [-4.5, -3.5, 0.0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.196\n', encoding='utf-8')
    return pixels
