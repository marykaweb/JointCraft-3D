"""Create isolated review artifacts; never opens or edits the user's scene."""
from pathlib import Path
import sys
import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import jointcraft as jc

bpy.ops.wm.read_factory_settings(use_empty=True)
jc.register()
scene = bpy.context.scene
bpy.ops.mesh.primitive_cube_add(size=.02)
pin = bpy.context.object
pin.name = 'Part_A_Pin'
bpy.ops.mesh.primitive_cube_add(size=.02, location=(.02, 0, 0))
socket = bpy.context.object
socket.name = 'Part_B_Socket'
scene.jc_pin, scene.jc_socket = pin, socket
assert bpy.ops.jointcraft.create_auto() == {'FINISHED'}
for obj, color in ((pin, (.07, .4, .65, 1)), (socket, (.8, .3, .08, 1))):
    mat = bpy.data.materials.new(obj.name)
    mat.diffuse_color = color
    obj.data.materials.append(mat)
scene.render.engine = 'CYCLES'
scene.cycles.device = 'CPU'
scene.cycles.samples = 32
scene.render.resolution_x = 640
scene.render.resolution_y = 640
scene.render.resolution_percentage = 100
scene.world = bpy.data.worlds.new('Review World')
scene.world.color = (.35, .35, .35)
bpy.ops.object.camera_add()
camera = bpy.context.object
scene.camera = camera
camera.data.type = 'ORTHO'
camera.data.ortho_scale = .041
camera.data.clip_start = .0001
camera.data.clip_end = 10
bpy.ops.object.light_add(type='AREA', location=(.01, -.04, .06))
light = bpy.context.object
light.data.energy = .4
light.data.shape = 'DISK'
light.data.size = .05
out = ROOT / 'artifacts'
out.mkdir(exist_ok=True)
for name, obj, hidden, position in (
        ('pin', pin, socket, (.06, -.04, .032)),
        ('socket', socket, pin, (-.04, -.04, .032))):
    obj.hide_render = False
    hidden.hide_render = True
    camera.location = position
    camera.rotation_euler = (obj.location - camera.location).to_track_quat('-Z', 'Y').to_euler()
    scene.render.filepath = str(out / (name + '.png'))
    bpy.ops.render.render(write_still=True)
pin.hide_render = socket.hide_render = False
bpy.ops.wm.save_as_mainfile(filepath=str(out / 'jointcraft-0.2.2-demo.blend'))
