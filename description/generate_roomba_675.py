#!/usr/bin/env python3
"""Build the stock Roomba 675 visual assets from the measured Xacro and photo profiles."""

from collections import defaultdict
from dataclasses import dataclass, field
from math import cos, pi, radians, sin, sqrt
from pathlib import Path
import shutil
import subprocess
from tempfile import TemporaryDirectory
from xml.etree import ElementTree as ET


DESCRIPTION = Path(__file__).resolve().parent
PACKAGE = DESCRIPTION.parent / 'ros2_ws/src/roomba_bringup'
GAZEBO_MODEL = PACKAGE / 'models/roomba_675'
STOCK_MASS_KG = 2.72  # Approximate assembled mass from the owner's 6 lb estimate.

# Linear RGB and roughness. The stock shell is black with a charcoal bumper.
MATERIALS = {
    'Shell': ((0.052, 0.049, 0.046), 0.42),
    'Deck': ((0.027, 0.024, 0.022), 0.22),
    'Bumper': ((0.039, 0.038, 0.037), 0.38),
    'SensorGlass': ((0.012, 0.009, 0.009), 0.17),
    'Seam': ((0.010, 0.010, 0.010), 0.66),
    'Underbody': ((0.060, 0.059, 0.056), 0.62),
    'Silver': ((0.50, 0.49, 0.44), 0.36),
    'LightSilver': ((0.70, 0.68, 0.61), 0.32),
    'Bin': ((0.085, 0.075, 0.065), 0.27),
    'Rubber': ((0.022, 0.023, 0.022), 0.82),
    'Tread': ((0.039, 0.040, 0.038), 0.75),
    'CasterWhite': ((0.66, 0.63, 0.54), 0.57),
    'BrushFrame': ((0.19, 0.20, 0.17), 0.53),
    'BrushFibers': ((0.21, 0.15, 0.095), 0.82),
    'BrushYellow': ((0.67, 0.44, 0.025), 0.46),
}


@dataclass
class Mesh:
    points: list = field(default_factory=list)
    triangles: list = field(default_factory=list)

    def face(self, *points):
        start = len(self.points)
        self.points.extend(points)
        self.triangles.extend((start, start + i, start + i + 1) for i in range(1, len(points) - 1))

    def append(self, other):
        start = len(self.points)
        self.points.extend(other.points)
        self.triangles.extend(tuple(start + index for index in tri) for tri in other.triangles)

    def normals(self):
        for ia, ib, ic in self.triangles:
            a, b, c = (self.points[i] for i in (ia, ib, ic))
            u = tuple(y - x for x, y in zip(a, b))
            v = tuple(y - x for x, y in zip(a, c))
            cross = (u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0])
            norm = sqrt(sum(value**2 for value in cross))
            yield tuple(value / norm for value in cross)


class Parts:
    def __init__(self):
        self.parts = defaultdict(Mesh)

    def add(self, name, material, mesh):
        self.parts[name, material].append(mesh)


def profile(section, start=0, end=360, steps=96, center=(0, 0, 0), axle=False):
    """Sweep a counterclockwise closed radius/height section around +Z or +Y."""
    mesh = Mesh()
    angles = [radians(start + (end - start) * i / steps) for i in range(steps + 1)]

    def point(angle, dimensions):
        radius, height = dimensions
        x, y, z = radius * cos(angle), radius * sin(angle), height
        if axle:
            x, y, z = x, -z, y
        return center[0] + x, center[1] + y, center[2] + z

    for a, b in zip(angles, angles[1:]):
        for lower, upper in zip(section, section[1:] + section[:1]):
            if lower[0] == upper[0] == 0:
                continue
            face = (point(a, lower), point(b, lower), point(b, upper), point(a, upper))
            mesh.face(*dict.fromkeys(face))
    if end - start < 360:
        mesh.face(*(point(angles[0], dim) for dim in section))
        mesh.face(*(point(angles[-1], dim) for dim in reversed(section)))
    return mesh


def cylinder(radius, low, high, center=(0, 0, 0), axle=False, steps=48):
    return profile([(0, low), (radius, low), (radius, high), (0, high)],
                   steps=steps, center=center, axle=axle)


def prism(outline, bottom, top):
    """Extrude a convex counterclockwise XY outline."""
    mesh = Mesh()
    low = [(x, y, bottom) for x, y in outline]
    high = [(x, y, top) for x, y in outline]
    mesh.face(*reversed(low))
    mesh.face(*high)
    for i in range(len(outline)):
        j = (i + 1) % len(outline)
        mesh.face(low[i], low[j], high[j], high[i])
    return mesh


def box(center, size, heading=0):
    cx, cy, cz = center
    x, y, z = (value / 2 for value in size)
    angle = radians(heading)
    outline = [(cx + u*cos(angle) - v*sin(angle), cy + u*sin(angle) + v*cos(angle))
               for u, v in [(-x, -y), (x, -y), (x, y), (-x, y)]]
    return prism(outline, cz - z, cz + z)


def clip_polygon(outline, axis, edge, direction):
    clipped = []
    for a, b in zip(outline[-1:] + outline[:-1], outline):
        da, db = direction * (a[axis] - edge), direction * (b[axis] - edge)
        if da * db < 0:
            t = da / (da - db)
            clipped.append(tuple(x + t * (y - x) for x, y in zip(a, b)))
        if db >= 0:
            clipped.append(b)
    return list(dict.fromkeys(clipped))


def underside_plate(radius, clearance, openings):
    """Tile the underside disk around rectangular wheel wells and the brush bay."""
    angles = [2*pi*i/96 for i in range(96)]
    disk = [(radius*cos(a), radius*sin(a)) for a in angles]
    xs = sorted({-radius, radius} | {x for a, b, _, _ in openings for x in (a, b)})
    ys = sorted({-radius, radius} | {y for _, _, a, b in openings for y in (a, b)})
    mesh = Mesh()
    for left, right in zip(xs, xs[1:]):
        for low, high in zip(ys, ys[1:]):
            if any(a <= left and right <= b and c <= low and high <= d for a, b, c, d in openings):
                continue
            panel = disk
            for axis, edge, sense in ((0, left, 1), (0, right, -1), (1, low, 1), (1, high, -1)):
                panel = clip_polygon(panel, axis, edge, sense)
            if len(panel) >= 3:
                mesh.append(prism(panel, clearance, clearance + 11))
    return mesh


# Small molded lettering, 5x7 bitmaps encoded as one integer per row.
GLYPHS = {
    'A': (14,17,17,31,17,17,17), 'B': (30,17,17,30,17,17,30),
    'C': (14,17,16,16,16,17,14), 'D': (30,17,17,17,17,17,30),
    'E': (31,16,16,30,16,16,31), 'I': (31,4,4,4,4,4,31),
    'K': (17,18,20,24,20,18,17), 'L': (16,16,16,16,16,16,31),
    'M': (17,27,21,21,17,17,17), 'N': (17,25,21,19,17,17,17),
    'O': (14,17,17,17,17,17,14), 'P': (30,17,17,30,16,16,16),
    'R': (30,17,17,30,20,18,17), 'S': (15,16,16,14,1,1,30),
    'T': (31,4,4,4,4,4,4),
}


def lettering(text, center, size):
    mesh = Mesh()
    cx, cy, cz = center
    for i, char in enumerate(text):
        for r, row in enumerate(GLYPHS[char]):
            for c in range(5):
                if row & (1 << (4 - c)):
                    x = cx + (3 - r) * size
                    y = cy + ((len(text) * 6 - 2) / 2 - (i * 6 + c)) * size
                    inset = size / 2
                    mesh.face((x-inset, y-inset, cz), (x+inset, y-inset, cz),
                              (x+inset, y+inset, cz), (x-inset, y+inset, cz))
    return mesh


def build_body(radius, height, clearance, wheel_y, wheel_radius, wheel_width, caster_x):
    parts = Parts()
    add = parts.add
    deck = height - 6
    add('StructuralShell', 'Shell', cylinder(radius-8, clearance+15, deck-2))
    add('LowerChassisRim', 'Underbody', profile([
        (radius-28, clearance), (radius-20, clearance), (radius-12, clearance+4),
        (radius-5, clearance+14), (radius-7, clearance+18), (radius-28, clearance+11),
    ]))
    bump = [(radius-7, 21), (radius-3, 20), (radius-0.8, 23), (radius, 29),
            (radius, deck-12), (radius-1.4, deck-6), (radius-4.5, deck-2), (radius-9, deck-0.7)]
    add('FrontBumper', 'Bumper', profile(bump, -88, 88, steps=56))
    for start, end in ((90, 136), (224, 270)):
        add('RearShell', 'Shell', profile(bump, start, end, steps=18))
    add('DustbinOuter', 'Bin', profile(bump, 137, 223, steps=32))
    add('BumperOptics', 'SensorGlass', profile([
        (radius-0.85, 41), (radius+0.05, 42), (radius+0.05, 56),
        (radius-0.85, 57), (radius-2.3, 57), (radius-2.3, 41),
    ], -67, 67, steps=48))

    add('TopDeck', 'Deck', profile([(0, deck-3), (radius-9, deck-3),
                                    (radius-9, deck-0.9), (radius-10.5, deck), (0, deck)]))
    add('HandleSeam', 'Seam', profile([(101, deck), (124, deck), (124, deck+0.6),
                                      (101, deck+0.6)], -67, 67, steps=40))
    add('CarryHandle', 'Shell', profile([(104, deck+0.5), (121, deck+0.5),
                                        (121, deck+0.9), (120, deck+1.4),
                                        (105, deck+1.4), (104, deck+0.9)], -67, 67, steps=40))
    for angle in (-67, 67):
        center = (112.5*cos(radians(angle)), 112.5*sin(radians(angle)), 0)
        add('CarryHandle', 'Shell', cylinder(8.45, deck+0.55, deck+1.35, center, steps=24))
    controls = (1, 0, 0)
    add('ControlBezel', 'Bumper', profile([(0, deck), (78, deck), (78, deck+0.65),
                                           (76.5, deck+1.35), (0, deck+1.35)], center=controls))
    add('ControlPanel', 'Deck', cylinder(75, deck+1.35, deck+1.52, controls, steps=96))
    add('ButtonBezel', 'Seam', cylinder(30.1, deck+1.52, deck+2.0, controls))
    add('SpotDockButtons', 'Silver', cylinder(28.1, deck+2.0, deck+2.35, controls))
    add('CleanButtonRim', 'Seam', cylinder(18, deck+2.35, deck+2.65, controls))
    add('CleanButton', 'LightSilver', cylinder(16.5, deck+2.65, deck+3.0, controls))
    for word, center, size in (
        ('CLEAN', (1, 0, deck+3.015), 0.8), ('SPOT', (24, 0, deck+2.365), 0.60),
        ('DOCK', (-22, 0, deck+2.365), 0.60),
    ):
        add('ButtonLabels', 'Shell', lettering(word, center, size))
    add('DeckLabels', 'LightSilver', lettering('IROBOT', (-101, 0, deck+0.015), 1.05))
    add('DeckLabels', 'LightSilver', lettering('ROOMBA', (-112, 0, deck+0.015), 0.65))
    add('DustbinRelease', 'Shell', cylinder(15, deck, deck+1.0, (-146, 0, 0)))
    add('IRReceiverBase', 'Bumper', cylinder(12, deck-0.6, deck+1.1, (radius-13, 0, 0)))
    add('IRReceiver', 'SensorGlass', profile([(0, deck), (9.6, deck), (9.6, height-1.2),
                                             (8.4, height), (0, height)], center=(radius-13, 0, 0)))

    # Rear exhaust recessed into the rear bin arc, represented by curved grille bars.
    add('ExhaustInset', 'Underbody', profile([(radius-2, 28), (radius+0.1, 28),
                                            (radius+0.1, 59), (radius-2, 59)], 152, 208, steps=22))
    for z in range(32, 57, 4):
        add('ExhaustSlots', 'Seam', profile([(radius, z), (radius+0.23, z),
                                           (radius+0.23, z+1.8), (radius, z+1.8)], 156, 204, steps=20))

    opening_half = wheel_width/2 + 5
    wheel_well = (-wheel_radius-6, wheel_radius+6, wheel_y-opening_half, wheel_y+opening_half)
    plate = underside_plate(radius-23, clearance, [
        wheel_well, (wheel_well[0], wheel_well[1], -wheel_well[3], -wheel_well[2]),
        (-88, 22, -105, 105), (caster_x-19, caster_x+19, -17, 17),
    ])
    add('UndersidePlate', 'Underbody', plate)
    add('CasterSocket', 'Seam', profile([(13, 13), (19, 13), (20, 18), (13, 20)],
                                       center=(caster_x, 0, 0), steps=48))
    add('BrushBay', 'Seam', box((-33, 0, 26), (110, 210, 6)))
    for y in (-101, 101):
        add('BrushFrame', 'BrushFrame', box((-33, y, 15), (111, 8, 9)))
        add('BrushRelease', 'BrushYellow', box((-78, y, 11.5), (13, 8, 5)))
    for x in (-87, 21):
        add('BrushFrame', 'BrushFrame', box((x, 0, 16), (8, 205, 9)))
    for y in (-75, -38, 0, 38, 75):
        add('BrushGuard', 'Silver', box((-33, y, 6), (105, 1.2, 1.4)))
    for x, name, material, core, outside in (
        (1, 'BeaterRoller', 'Rubber', 8, 12), (-48, 'BristleRoller', 'BrushFibers', 8.5, 13),
    ):
        center = (x, 0, 18)
        add(name, 'Rubber', cylinder(core, -94, 94, center, axle=True, steps=40))
        for n in range(4):
            for i in range(12):
                y = -94 + i * (188 / 12)
                a = 90*n + 8*i
                section = [(core-0.5, y), (outside, y), (outside, y+15), (core-0.5, y+15)]
                add(name, material, profile(section, a, a+14, steps=2, center=center, axle=True))

    add('BinUndertray', 'Bin', profile([(88, 12), (radius-31, 12),
                                      (radius-23, 15), (radius-18, 21), (88, 21)], 135, 225, steps=30))
    for y in (-43, 43):
        add('ChargingContacts', 'Silver', box((132, y, 9.7), (17, 15, 2)))
    for x, y, heading in ((144, -40, -15), (144, 40, 15), (72, -135, -61), (72, 135, 61)):
        add('CliffSensorWindows', 'SensorGlass', box((x, y, clearance-0.55), (10, 18, 2), heading))
    for x, y in ((67, -65), (67, 65), (-54, -132), (-54, 132), (123, -75), (123, 75)):
        add('Fasteners', 'Seam', cylinder(5, clearance-0.15, clearance+0.25, (x, y, 0), steps=16))
        add('Fasteners', 'Silver', cylinder(2.5, clearance-0.4, clearance-0.15, (x, y, 0), steps=16))

    sx, sy = 93, -105
    add('SideBrushHub', 'BrushYellow', cylinder(13, 7, 15, (sx, sy, 0)))
    add('SideBrushFastener', 'Silver', cylinder(4, 6.7, 7, (sx, sy, 0), steps=18))
    for heading in (-65, 55, 175):
        angle = radians(heading)
        add('SideBrushArms', 'BrushYellow', box((sx+16*cos(angle), sy+16*sin(angle), 8.5),
                                               (26, 4.2, 3.1), heading))
        for spread in (-9, -5, 0, 5, 9):
            a = radians(heading+spread)
            add('SideBrushBristles', 'CasterWhite', box((sx+38*cos(a), sy+38*sin(a), 3.5),
                                                      (28, 0.95, 2.0), heading+spread))
    return parts


def build_wheel(radius, width):
    parts = Parts()
    low, high = -width/2, width/2
    parts.add('Tire', 'Rubber', profile([
        (0, low+0.5), (radius-4, low+0.5), (radius-1.5, low+1.6),
        (radius-1.5, high-1.6), (radius-4, high-0.5), (0, high-0.5),
    ], axle=True, steps=72))
    for i in range(36):
        a = 10*i
        for j, y in enumerate((low, low+width*0.35, low+width*0.69)):
            section = [(radius-2.5, y), (radius-0.55, y), (radius, y+0.55),
                       (radius, y+width*0.29-0.55), (radius-0.55, y+width*0.29),
                       (radius-2.5, y+width*0.29)]
            parts.add('TreadLugs', 'Tread', profile(section, a+j*2-2, a+j*2+4, steps=3, axle=True))
    for y in (low, high):
        parts.add('Hub', 'Underbody', cylinder(radius*0.58, y-0.42, y+0.42, axle=True))
        parts.add('HubCap', 'Rubber', cylinder(radius*0.23, y-0.62, y+0.62, axle=True, steps=28))
    return parts


def build_caster(radius):
    parts = Parts()
    width = 0.67 * radius
    section = [(0, -width), (0.84*radius, -width), (radius, -0.47*width),
               (radius, 0.47*width), (0.84*radius, width), (0, width)]
    parts.add('CasterTire', 'Rubber', profile(section, 0, 180, steps=24, axle=True))
    parts.add('CasterTire', 'CasterWhite', profile(section, 180, 360, steps=24, axle=True))
    parts.add('CasterAxle', 'Underbody', cylinder(radius*0.27, -width-1, width+1, axle=True, steps=24))
    return parts


def numbers(values):
    return ' '.join(f'{number:.8g}' for number in values)


def usd_tuple(values):
    return '(' + ', '.join(numbers((value,)) for value in values) + ')'


def usd_array(values):
    return '[' + ', '.join(values) + ']'


def write_usda(path, components):
    lines = ['#usda 1.0', '(', '    defaultPrim = "Roomba675"',
             '    metersPerUnit = 1', '    kilogramsPerUnit = 1', '    upAxis = "Z"', ')', '',
             'def Xform "Roomba675" (', '    kind = "component"',
             '    prepend apiSchemas = ["PhysicsMassAPI"]', ')', '{',
             f'    float physics:mass = {STOCK_MASS_KG} (',
             '        documentation = "Approximate assembled mass from a 6 lb estimate."', '    )',
             '    def Scope "Materials"', '    {']
    for name, (color, roughness) in MATERIALS.items():
        lines.extend([f'        def Material "{name}"', '        {',
                      f'            token outputs:surface.connect = </Roomba675/Materials/{name}/Surface.outputs:surface>',
                      '            def Shader "Surface"', '            {',
                      '                uniform token info:id = "UsdPreviewSurface"',
                      f'                color3f inputs:diffuseColor = {usd_tuple(color)}',
                      f'                float inputs:roughness = {roughness}',
                      '                token outputs:surface', '            }', '        }'])
    lines.append('    }')
    for component, parts, location in components:
        lines.extend([f'    def Xform "{component}"', '    {',
                      f'        double3 xformOp:translate = {usd_tuple(v/1000 for v in location)}',
                      '        uniform token[] xformOpOrder = ["xformOp:translate"]'])
        for (name, material), mesh in parts.parts.items():
            points = [tuple(v/1000 for v in point) for point in mesh.points]
            extent = [tuple(f(point[i] for point in points) for i in range(3)) for f in (min, max)]
            indices = usd_array(str(index) for face in mesh.triangles for index in face)
            lines.extend([f'        def Mesh "{name}_{material}" (prepend apiSchemas = ["MaterialBindingAPI"])',
                          '        {', '            uniform token subdivisionScheme = "none"',
                          f'            int[] faceVertexCounts = {usd_array("3" for _ in mesh.triangles)}',
                          f'            int[] faceVertexIndices = {indices}',
                          f'            point3f[] points = {usd_array(map(usd_tuple, points))}',
                          f'            float3[] extent = {usd_array(map(usd_tuple, extent))}',
                          f'            normal3f[] normals = {usd_array(map(usd_tuple, mesh.normals()))} (interpolation = "uniform")',
                          f'            color3f[] primvars:displayColor = [{usd_tuple(MATERIALS[material][0])}]',
                          f'            rel material:binding = </Roomba675/Materials/{material}>', '        }'])
        lines.append('    }')
    path.write_text('\n'.join(lines + ['}', '']), encoding='utf-8')


def source(mesh, source_id, values):
    values = list(values)
    element = ET.SubElement(mesh, 'source', id=source_id)
    ET.SubElement(element, 'float_array', id=source_id+'-array', count=str(3*len(values))).text = (
        numbers(v for value in values for v in value))
    technique = ET.SubElement(element, 'technique_common')
    accessor = ET.SubElement(technique, 'accessor', source='#'+source_id+'-array', count=str(len(values)), stride='3')
    for axis in 'XYZ':
        ET.SubElement(accessor, 'param', name=axis, type='float')


def write_dae(path, parts):
    root = ET.Element('COLLADA', xmlns='http://www.collada.org/2005/11/COLLADASchema', version='1.4.1')
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'created').text = '2026-11-27T00:00:00Z'
    ET.SubElement(asset, 'modified').text = '2026-11-27T00:00:00Z'
    ET.SubElement(asset, 'unit', name='meter', meter='1')
    ET.SubElement(asset, 'up_axis').text = 'Z_UP'
    effects = ET.SubElement(root, 'library_effects')
    materials = ET.SubElement(root, 'library_materials')
    for name in dict.fromkeys(material for _, material in parts.parts):
        color, roughness = MATERIALS[name]
        effect = ET.SubElement(effects, 'effect', id=name+'-effect')
        profile_element = ET.SubElement(effect, 'profile_COMMON')
        shader = ET.SubElement(ET.SubElement(profile_element, 'technique', sid='common'), 'phong')
        for property_name, values in (('ambient', (*color, 1)), ('diffuse', (*color, 1)),
                                      ('specular', (0.18, 0.18, 0.18, 1))):
            ET.SubElement(ET.SubElement(shader, property_name), 'color').text = numbers(values)
        ET.SubElement(ET.SubElement(shader, 'shininess'), 'float').text = numbers(((1-roughness)*70,))
        material = ET.SubElement(materials, 'material', id=name, name=name)
        ET.SubElement(material, 'instance_effect', url='#'+name+'-effect')

    geometries = ET.SubElement(root, 'library_geometries')
    scene = ET.SubElement(ET.SubElement(root, 'library_visual_scenes'), 'visual_scene', id='Scene')
    for (name, material), shape in parts.parts.items():
        mesh_id = name + '-' + material
        geometry = ET.SubElement(geometries, 'geometry', id=mesh_id, name=name)
        mesh = ET.SubElement(geometry, 'mesh')
        source(mesh, mesh_id+'-points', (tuple(v/1000 for v in point) for point in shape.points))
        source(mesh, mesh_id+'-normals', shape.normals())
        vertices = ET.SubElement(mesh, 'vertices', id=mesh_id+'-vertices')
        ET.SubElement(vertices, 'input', semantic='POSITION', source='#'+mesh_id+'-points')
        triangles = ET.SubElement(mesh, 'triangles', count=str(len(shape.triangles)), material=material)
        ET.SubElement(triangles, 'input', semantic='VERTEX', source='#'+mesh_id+'-vertices', offset='0')
        ET.SubElement(triangles, 'input', semantic='NORMAL', source='#'+mesh_id+'-normals', offset='1')
        ET.SubElement(triangles, 'p').text = ' '.join(
            f'{index} {face_id}' for face_id, face in enumerate(shape.triangles) for index in face)
        instance = ET.SubElement(ET.SubElement(scene, 'node', id=mesh_id+'-node'), 'instance_geometry', url='#'+mesh_id)
        bind = ET.SubElement(ET.SubElement(instance, 'bind_material'), 'technique_common')
        ET.SubElement(bind, 'instance_material', symbol=material, target='#'+material)
    ET.SubElement(ET.SubElement(root, 'scene'), 'instance_visual_scene', url='#Scene')
    write_xml(path, root)


def write_xml(path, root):
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def write_gazebo_model(components, radius, height, clearance, wheel_radius, wheel_width, caster_radius):
    config = ET.Element('model')
    ET.SubElement(config, 'name').text = 'Stock Roomba 675'
    ET.SubElement(config, 'version').text = '1.0'
    ET.SubElement(config, 'sdf', version='1.9').text = 'model.sdf'
    ET.SubElement(config, 'description').text = 'Colored stock Roomba 675 for static scene placement.'
    write_xml(GAZEBO_MODEL / 'model.config', config)

    sdf = ET.Element('sdf', version='1.9')
    model = ET.SubElement(sdf, 'model', name='roomba_675')
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='chassis')
    files = ('body.dae', 'drive_wheel.dae', 'drive_wheel.dae', 'caster.dae')
    for (name, _, location), filename in zip(components, files):
        visual = ET.SubElement(link, 'visual', name=name)
        ET.SubElement(visual, 'pose').text = numbers([*(v/1000 for v in location), 0, 0, 0])
        geometry = ET.SubElement(visual, 'geometry')
        ET.SubElement(ET.SubElement(geometry, 'mesh'), 'uri').text = 'meshes/' + filename

    body_z = (height + clearance) / 2
    collisions = [
        ('Body', (0, 0, body_z, 0, 0, 0), 'cylinder', {'radius': radius, 'length': height-clearance}),
        ('FrontCaster', (*components[3][2], 0, 0, 0), 'sphere', {'radius': caster_radius}),
    ]
    for name, _, location in components[1:3]:
        collisions.append((name, (*location, pi/2, 0, 0), 'cylinder',
                           {'radius': wheel_radius, 'length': wheel_width}))
    for name, pose, shape, fields in collisions:
        collision = ET.SubElement(link, 'collision', name=name)
        ET.SubElement(collision, 'pose').text = numbers([*(v/1000 for v in pose[:3]), *pose[3:]])
        primitive = ET.SubElement(ET.SubElement(collision, 'geometry'), shape)
        for key, value in fields.items():
            ET.SubElement(primitive, key).text = numbers((value/1000,))
    write_xml(GAZEBO_MODEL / 'model.sdf', sdf)


def main():
    usdcat = shutil.which('usdcat')
    if usdcat is None:
        raise SystemExit('OpenUSD usdcat is required to pack the binary USD file.')
    source_urdf = PACKAGE / 'urdf/roomba_675.urdf.xacro'
    root = ET.parse(source_urdf).getroot()
    dimensions = {element.get('name'): float(element.get('value')) * 1000
                  for element in root.findall('{http://ros.org/wiki/xacro}property')
                  if element.get('name') in {'body_diameter', 'top_height', 'ground_clearance',
                                             'wheel_radius', 'tread_width', 'tread_inside_offset',
                                             'caster_radius', 'caster_from_front'}}
    body_radius = dimensions['body_diameter'] / 2
    height = dimensions['top_height']
    clearance = dimensions['ground_clearance']
    wheel_radius = dimensions['wheel_radius']
    width = dimensions['tread_width']
    wheel_y = dimensions['tread_inside_offset'] + width / 2
    caster_radius = dimensions['caster_radius']
    caster_x = body_radius - dimensions['caster_from_front']

    body = build_body(body_radius, height, clearance, wheel_y, wheel_radius, width, caster_x)
    wheel = build_wheel(wheel_radius, width)
    caster = build_caster(caster_radius)
    components = [
        ('Chassis', body, (0, 0, 0)), ('LeftDriveWheel', wheel, (0, wheel_y, wheel_radius)),
        ('RightDriveWheel', wheel, (0, -wheel_y, wheel_radius)), ('FrontCaster', caster, (caster_x, 0, caster_radius)),
    ]
    with TemporaryDirectory() as scratch:
        usda = Path(scratch) / 'roomba_675.usda'
        usd = DESCRIPTION / 'roomba_675.usd'
        write_usda(usda, components)
        subprocess.run([usdcat, '--usdFormat', 'usdc', '-o', str(usd), str(usda)], check=True)
        usd.chmod(0o644)
    meshes = GAZEBO_MODEL / 'meshes'
    meshes.mkdir(parents=True, exist_ok=True)
    for name, parts in (('body', body), ('drive_wheel', wheel), ('caster', caster)):
        write_dae(meshes / (name + '.dae'), parts)
    write_gazebo_model(components, body_radius, height, clearance, wheel_radius, width, caster_radius)


if __name__ == '__main__':
    main()
