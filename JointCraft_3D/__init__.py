bl_info = {
    "name": "JointCraft 3D",
    "author": "JointCraft 3D",
    "version": (0, 2, 0),
    "blender": (5, 2, 0),
    "location": "View3D > Sidebar > JointCraft",
    "description": "Automatically place a fixed cylindrical pin and socket between touching meshes",
    "category": "3D View",
}

import bpy
import math
from mathutils import Vector
from bpy.props import PointerProperty, FloatProperty, BoolProperty


def mm_to_bu(context, mm):
    scale = context.scene.unit_settings.scale_length or 1.0
    return mm * 0.001 / scale


def bounds_world(obj):
    points = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
    return ([min(p[i] for p in points) for i in range(3)],
            [max(p[i] for p in points) for i in range(3)])


def find_interface(context, pin, socket, tolerance):
    pmin, pmax = bounds_world(pin)
    smin, smax = bounds_world(socket)
    choices = []
    for axis in range(3):
        others = [i for i in range(3) if i != axis]
        overlaps = [(max(pmin[i], smin[i]), min(pmax[i], smax[i])) for i in others]
        if any(hi - lo <= tolerance for lo, hi in overlaps):
            continue
        for sign, face_pin, face_socket in ((1, pmax[axis], smin[axis]),
                                            (-1, pmin[axis], smax[axis])):
            gap = face_socket - face_pin if sign == 1 else face_pin - face_socket
            if abs(gap) <= tolerance:
                area = math.prod(hi - lo for lo, hi in overlaps)
                choices.append((abs(gap), -area, axis, sign, face_pin, overlaps,
                                pmin, pmax, smin, smax))
    if not choices:
        raise ValueError("No touching axis-aligned faces found. Move meshes so their faces meet.")
    choices.sort(key=lambda v: (v[0], v[1]))
    return choices[0]


def cylinder_mesh(name, origin, axis, sign, start, end, radius, segments=48):
    other = [i for i in range(3) if i != axis]
    vertices = []
    for t in (start, end):
        for j in range(segments):
            theta = 2 * math.pi * j / segments
            pt = list(origin)
            pt[axis] += sign * t
            pt[other[0]] += radius * math.cos(theta)
            pt[other[1]] += radius * math.sin(theta)
            vertices.append(tuple(pt))
    faces = [tuple(reversed(range(segments))), tuple(range(segments, 2 * segments))]
    for j in range(segments):
        k = (j + 1) % segments
        faces.append((j, k, segments + k, segments + j))
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    return mesh


def boolean_mesh(context, target, cutter, operation):
    modifier = target.modifiers.new("JC temporary " + operation, 'BOOLEAN')
    modifier.operation = operation
    modifier.solver = 'EXACT'
    modifier.object = cutter
    try:
        depsgraph = context.evaluated_depsgraph_get()
        evaluated = target.evaluated_get(depsgraph)
        new_mesh = bpy.data.meshes.new_from_object(evaluated, preserve_all_data_layers=True,
                                                   depsgraph=depsgraph)
        if new_mesh is None:
            raise RuntimeError("Boolean evaluation failed")
    finally:
        target.modifiers.remove(modifier)
    old = target.data
    target.data = new_mesh
    if old.users == 0:
        bpy.data.meshes.remove(old)


class JOINTCRAFT_OT_create(bpy.types.Operator):
    bl_idname = "jointcraft.create_auto"
    bl_label = "Create Auto Pin + Socket"
    bl_description = "Find a shared face, calculate dimensions and cut a fixed pin/socket"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        scene = context.scene
        return (context.mode == 'OBJECT' and scene.jc_pin is not None and
                scene.jc_socket is not None and scene.jc_pin != scene.jc_socket)

    def execute(self, context):
        scene = context.scene
        pin, socket = scene.jc_pin, scene.jc_socket
        if pin.type != 'MESH' or socket.type != 'MESH':
            self.report({'ERROR'}, "Both objects must be meshes")
            return {'CANCELLED'}
        if pin.library or socket.library or pin.data.library or socket.data.library:
            self.report({'ERROR'}, "Linked meshes cannot be edited")
            return {'CANCELLED'}
        if any(abs(v) < 1e-9 for v in pin.scale) or any(abs(v) < 1e-9 for v in socket.scale):
            self.report({'ERROR'}, "Object scale must not be zero")
            return {'CANCELLED'}
        clearance = mm_to_bu(context, scene.jc_clearance_mm)
        wall = mm_to_bu(context, scene.jc_wall_mm)
        tol = mm_to_bu(context, scene.jc_tolerance_mm)
        try:
            _, _, axis, sign, face, overlaps, pmin, pmax, smin, smax = find_interface(
                context, pin, socket, tol)
            cross_width = min(hi - lo for lo, hi in overlaps)
            socket_thickness = smax[axis] - smin[axis]
            pin_thickness = pmax[axis] - pmin[axis]
            radius = min(cross_width * 0.20, mm_to_bu(context, 4.0),
                         cross_width * 0.5 - wall - clearance)
            length = min(socket_thickness * 0.45, mm_to_bu(context, 6.0),
                         socket_thickness - wall - clearance)
            embed = min(length * 0.6, pin_thickness - wall)
            if radius < mm_to_bu(context, 1.0) or length <= clearance or embed <= 0:
                raise ValueError("Parts are too small for a safe connector with these wall/clearance values")
            center = [0.0] * 3
            center[axis] = face
            for index, (lo, hi) in zip([i for i in range(3) if i != axis], overlaps):
                center[index] = (lo + hi) * 0.5
            # World-space cutter; target object transforms remain unchanged.
            pin_mesh = cylinder_mesh("JC_Pin_Temp", center, axis, sign, -embed, length, radius)
            socket_mesh = cylinder_mesh("JC_Socket_Temp", center, axis, sign,
                                        -max(tol, clearance * 0.15),
                                        length + clearance, radius + clearance)
            pin_cutter = bpy.data.objects.new("JC_Pin_Temp", pin_mesh)
            socket_cutter = bpy.data.objects.new("JC_Socket_Temp", socket_mesh)
            context.collection.objects.link(pin_cutter)
            context.collection.objects.link(socket_cutter)
            try:
                # Boolean works in world coordinates via cutter object matrix.
                # If target is transformed, world-space cutters remain correct.
                boolean_mesh(context, pin, pin_cutter, 'UNION')
                boolean_mesh(context, socket, socket_cutter, 'DIFFERENCE')
            finally:
                bpy.data.objects.remove(pin_cutter, do_unlink=True)
                bpy.data.objects.remove(socket_cutter, do_unlink=True)
            self.report({'INFO'}, "Joint created: diameter %.2f mm, length %.2f mm" %
                        (radius * 2 / mm_to_bu(context, 1), length / mm_to_bu(context, 1)))
            return {'FINISHED'}
        except Exception as exc:
            self.report({'ERROR'}, "JointCraft: " + str(exc))
            return {'CANCELLED'}


class JOINTCRAFT_PT_main(bpy.types.Panel):
    bl_label = "JointCraft 3D v0.2"
    bl_idname = "JOINTCRAFT_PT_main"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'JointCraft'

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        layout.label(text="Auto: touching flat faces")
        layout.prop(scene, "jc_pin", text="Pin object")
        layout.prop(scene, "jc_socket", text="Socket object")
        layout.separator()
        layout.prop(scene, "jc_clearance_mm")
        layout.prop(scene, "jc_wall_mm")
        advanced = layout.box()
        advanced.prop(scene, "jc_show_advanced", text="Advanced", icon='PREFERENCES')
        if scene.jc_show_advanced:
            advanced.prop(scene, "jc_tolerance_mm")
        layout.operator("jointcraft.create_auto", icon='MOD_BOOLEAN')
        layout.label(text="Dimensions calculated automatically", icon='INFO')
        layout.label(text="Works best with closed mesh cubes")


classes = (JOINTCRAFT_OT_create, JOINTCRAFT_PT_main)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.jc_pin = PointerProperty(name="Pin object", type=bpy.types.Object)
    bpy.types.Scene.jc_socket = PointerProperty(name="Socket object", type=bpy.types.Object)
    bpy.types.Scene.jc_clearance_mm = FloatProperty(name="Radial clearance (mm)", default=0.20, min=0.0, max=2.0, precision=2)
    bpy.types.Scene.jc_wall_mm = FloatProperty(name="Minimum wall (mm)", default=1.2, min=0.1, max=20.0, precision=2)
    bpy.types.Scene.jc_tolerance_mm = FloatProperty(name="Face tolerance (mm)", default=0.05, min=0.001, max=2.0, precision=3)
    bpy.types.Scene.jc_show_advanced = BoolProperty(name="Advanced", default=False)


def unregister():
    for name in ("jc_pin", "jc_socket", "jc_clearance_mm", "jc_wall_mm", "jc_tolerance_mm", "jc_show_advanced"):
        delattr(bpy.types.Scene, name)
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
