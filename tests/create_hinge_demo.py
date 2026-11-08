"""Generate assembled/moving/exploded review images and three millimetre STLs."""
from pathlib import Path
import math
import struct
import zipfile
import sys
import bpy
from mathutils import Vector, Matrix

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import jointcraft as jc
from jointcraft import hinges

OUT = ROOT / 'artifacts' / 'hinge-0.3.0'
OUT.mkdir(parents=True, exist_ok=True)
bpy.ops.wm.read_factory_settings(use_empty=True)
jc.register()
s = bpy.context.scene
parts = []
for name, x in (('Part A', -.01), ('Part B', .01)):
    bpy.ops.mesh.primitive_cube_add(size=.02, location=(x, 0, 0))
    obj = bpy.context.object
    obj.name = name
    obj.scale.z = .4
    parts.append(obj)
s.jc_joint_type = 'HINGE'
s.jc_hinge_a, s.jc_hinge_b = parts
p = hinges.parameters(bpy.context)
pin = hinges.build(bpy.context, p)
pin.name = 'Hinge Pin'
s.jc_hinge_preview = False
objects = [*parts, pin]


def stl_mm(obj, path):
    mesh = obj.data
    mesh.calc_loop_triangles()
    matrix = obj.matrix_world
    verts = [matrix @ vert.co * 1000 for vert in mesh.vertices]
    lowest = min(v.z for v in verts)
    with path.open('wb') as stream:
        stream.write(b'JointCraft 3D 0.3.0 - dimensions in millimetres'.ljust(80, b'\0'))
        stream.write(struct.pack('<I', len(mesh.loop_triangles)))
        for tri in mesh.loop_triangles:
            points = [verts[i] - Vector((0, 0, lowest)) for i in tri.vertices]
            normal = (points[1] - points[0]).cross(points[2] - points[0]).normalized()
            stream.write(struct.pack('<12fH', *normal, *points[0], *points[1], *points[2], 0))


for obj, name, color in zip(objects, ('part-a.stl', 'part-b.stl', 'pin.stl'),
                          ((.03, .35, .75, 1), (1, .3, .04, 1), (.08, .6, .16, 1))):
    stl_mm(obj, OUT / name)
    material = bpy.data.materials.new(obj.name)
    material.diffuse_color = color
    material.use_nodes = True
    material.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value = color
    material.node_tree.nodes['Principled BSDF'].inputs['Roughness'].default_value = .4
    obj.data.materials.clear()
    obj.data.materials.append(material)
(OUT / 'PRINT-README.txt').write_text(
    'JointCraft 3D 0.3.0 hinge print test. All STL coordinates are millimetres.\n'
    'Print parts separately; do not fuse or print as one assembled object.\n'
    'Bodies: 20 x 20 x 8 mm before hinge relief. Pin diameter: 3 mm.\n'
    'Radial gap: 0.25 mm; axial gap each side: 0.30 mm; hinge height: 8 mm.\n'
    'Review slicer supports below the upper outer knuckle of Part A.\n'
    'Clean supports and holes, align parts, insert headed pin from below.\n'
    'Check movement before fixing. To retain pin, bond exposed tip to Part A only.\n'
    'Keep adhesive away from Part B and axial gaps. Head alone is not a lock.\n'
    'Geometrically tested to +/-30 degrees on this pair; physical print is unverified.\n',
    encoding='utf-8')
with zipfile.ZipFile(OUT / 'jointcraft-hinge-print-test.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
    for name in ('part-a.stl', 'part-b.stl', 'pin.stl', 'PRINT-README.txt'):
        archive.write(OUT / name, name)
s.render.engine = 'CYCLES'
s.cycles.device = 'CPU'
s.cycles.samples = 48
s.render.resolution_x = 1000
s.render.resolution_y = 700
s.render.resolution_percentage = 100
s.world = bpy.data.worlds.new('Review World')
s.world.color = (.25, .25, .25)
bpy.ops.object.camera_add(location=(.045, -.065, .035))
camera = bpy.context.object
camera.rotation_euler = (-camera.location).to_track_quat('-Z', 'Y').to_euler()
camera.data.type = 'ORTHO'
camera.data.ortho_scale = .06
camera.data.clip_start = .0001
s.camera = camera
for position, power in (((-.02, -.03, .05), .4), ((.03, .02, .04), .3)):
    bpy.ops.object.light_add(type='AREA', location=position)
    light = bpy.context.object
    light.data.energy = power
    light.data.size = .05
    light.rotation_euler = (-light.location).to_track_quat('-Z', 'Y').to_euler()
bpy.context.view_layer.update()
bpy.ops.wm.save_as_mainfile(filepath=str(OUT / 'jointcraft-hinge-0.3.0.blend'))
original = parts[1].matrix_world.copy()
for name in ('neutral', '30deg', 'exploded'):
    parts[1].matrix_world = original
    pin.location = (0, 0, 0)
    if name == '30deg':
        parts[1].matrix_world = Matrix.Rotation(math.radians(30), 4, 'Z') @ original
    if name == 'exploded':
        parts[1].location.x += .008
        pin.location.z = -.013
    s.render.filepath = str(OUT / ('hinge-' + name + '.png'))
    bpy.ops.render.render(write_still=True)
