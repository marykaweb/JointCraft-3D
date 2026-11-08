bl_info = {
    "name": "JointCraft 3D",
    "author": "JointCraft 3D",
    "version": (0, 2, 1),
    "blender": (5, 2, 0),
    "location": "View3D > Sidebar > JointCraft",
    "description": "Automatically place a fixed cylindrical pin and socket between touching meshes",
    "category": "3D View",
}

import bpy
import math
from mathutils import Vector
from bpy.props import PointerProperty, FloatProperty, BoolProperty
from .geometry import interface_from_bounds


def mm_to_bu(context, mm):
    scale = context.scene.unit_settings.scale_length or 1.0
    return mm * 0.001 / scale


def bounds_world(obj):
    points = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
    return ([min(p[i] for p in points) for i in range(3)],
            [max(p[i] for p in points) for i in range(3)])


def find_interface(context, pin, socket, tolerance):
    for obj in (pin, socket):
        if obj is None or obj.type != 'MESH':
            raise ValueError("Choose two different mesh objects")
        if context.scene.objects.get(obj.name) != obj:
            raise ValueError("Object '%s' is not in the active scene" % obj.name)
        if context.view_layer.objects.get(obj.name) != obj:
            raise ValueError("Object '%s' is not in the active view layer" % obj.name)
        if not obj.data.vertices:
            raise ValueError("Object '%s' has no mesh vertices" % obj.name)
    if pin == socket:
        raise ValueError("Pin and socket must be different objects")
    # RNA location/scale writes can leave matrix_world stale until evaluation.
    context.view_layer.update()
    depsgraph = context.evaluated_depsgraph_get()
    pmin, pmax = bounds_world(pin.evaluated_get(depsgraph))
    smin, smax = bounds_world(socket.evaluated_get(depsgraph))
    try:
        return interface_from_bounds(pmin, pmax, smin, smax, tolerance)
    except ValueError as exc:
        details = ("Pin '%s': %s .. %s; Socket '%s': %s .. %s; "
                   "requested tolerance %.6g BU (%.4g mm). "
                   "Check assigned objects and world-axis-aligned face contact." %
                   (pin.name, pmin, pmax, socket.name, smin, smax, tolerance,
                    tolerance / mm_to_bu(context, 1.0)))
        raise ValueError("%s. %s" % (exc, details)) from exc


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
    # Swapping X/Z for a Y cylinder or reversing the axis flips handedness.
    if sign * (-1 if axis == 1 else 1) < 0:
        faces = [tuple(reversed(face)) for face in faces]
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
        if new_mesh is None or not new_mesh.vertices or not new_mesh.polygons:
            if new_mesh is not None:
                bpy.data.meshes.remove(new_mesh)
            raise RuntimeError("Boolean evaluation failed")
    finally:
        target.modifiers.remove(modifier)
    return new_mesh


def apply_joint(context, pin, socket, center, axis, sign, embed, length, radius,
                clearance, tolerance):
    """Prepare both Boolean results before replacing either source mesh."""
    cutters, temporary_meshes, results = [], [], []
    originals = (pin.data, socket.data)
    try:
        for name, start, end, r in (
                ('JC_Pin_Temp', -embed, length, radius),
                ('JC_Socket_Temp', -max(tolerance, clearance * 0.15),
                 length + clearance, radius + clearance)):
            mesh = cylinder_mesh(name, center, axis, sign, start, end, r)
            temporary_meshes.append(mesh)
            cutter = bpy.data.objects.new(name, mesh)
            cutters.append(cutter)
            context.scene.collection.objects.link(cutter)
        results.append(boolean_mesh(context, pin, cutters[0], 'UNION'))
        results.append(boolean_mesh(context, socket, cutters[1], 'DIFFERENCE'))
        try:
            pin.data, socket.data = results
        except Exception:
            pin.data, socket.data = originals
            raise
    finally:
        for cutter in cutters:
            bpy.data.objects.remove(cutter, do_unlink=True)
        for mesh in temporary_meshes + results:
            if mesh.users == 0:
                bpy.data.meshes.remove(mesh)
    # Keep shared source datablocks, and delete only unused originals on success.
    for mesh in set(originals):
        if mesh.users == 0:
            bpy.data.meshes.remove(mesh)


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
        if pin is None or socket is None or pin == socket:
            self.report({'ERROR'}, "Choose two different mesh objects")
            return {'CANCELLED'}
        if pin.type != 'MESH' or socket.type != 'MESH':
            self.report({'ERROR'}, "Both objects must be meshes")
            return {'CANCELLED'}
        if pin.library or socket.library or pin.data.library or socket.data.library:
            self.report({'ERROR'}, "Linked meshes cannot be edited")
            return {'CANCELLED'}
        if any(obj.modifiers or obj.data.shape_keys for obj in (pin, socket)):
            self.report({'ERROR'}, "Apply modifiers and resolve shape keys before creating a joint")
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
            apply_joint(context, pin, socket, center, axis, sign, embed, length,
                        radius, clearance, tol)
            self.report({'INFO'}, "Joint created: diameter %.2f mm, length %.2f mm" %
                        (radius * 2 / mm_to_bu(context, 1), length / mm_to_bu(context, 1)))
            return {'FINISHED'}
        except Exception as exc:
            self.report({'ERROR'}, "JointCraft: " + str(exc))
            return {'CANCELLED'}


class JOINTCRAFT_PT_main(bpy.types.Panel):
    bl_label = "JointCraft 3D v0.2.1"
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


def mesh_in_scene(scene, obj):
    return obj.type == 'MESH' and scene.objects.get(obj.name) == obj


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.jc_pin = PointerProperty(name="Pin object", type=bpy.types.Object, poll=mesh_in_scene)
    bpy.types.Scene.jc_socket = PointerProperty(name="Socket object", type=bpy.types.Object, poll=mesh_in_scene)
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
