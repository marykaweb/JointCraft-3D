bl_info = {
    "name": "JointCraft 3D",
    "author": "JointCraft 3D",
    "version": (0, 2, 2),
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


def find_interface(context, pin, socket, tolerance, refresh=True):
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
    if refresh:
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


def joint_parameters(context, refresh=True):
    """Single source of dimensions for preview and final Boolean cutters."""
    scene = context.scene
    pin, socket = scene.jc_pin, scene.jc_socket
    if pin is None or socket is None or pin == socket:
        raise ValueError("Choose two different mesh objects")
    if pin.type != 'MESH' or socket.type != 'MESH':
        raise ValueError("Both objects must be meshes")
    if pin.library or socket.library or pin.data.library or socket.data.library:
        raise ValueError("Linked meshes cannot be edited")
    if any(obj.modifiers or obj.data.shape_keys for obj in (pin, socket)):
        raise ValueError("Apply modifiers and resolve shape keys before creating a joint")
    if any(abs(v) < 1e-9 for obj in (pin, socket) for v in obj.scale):
        raise ValueError("Object scale must not be zero")
    clearance = mm_to_bu(context, scene.jc_clearance_mm)
    wall = mm_to_bu(context, scene.jc_wall_mm)
    tol = mm_to_bu(context, scene.jc_tolerance_mm)
    _, _, axis, sign, face, overlaps, pmin, pmax, smin, smax = find_interface(
        context, pin, socket, tol, refresh=refresh)
    cross_width = min(hi - lo for lo, hi in overlaps)
    socket_thickness = smax[axis] - smin[axis]
    pin_thickness = pmax[axis] - pmin[axis]
    max_radius = cross_width * .5 - wall - clearance
    max_length = socket_thickness - wall - clearance
    if scene.jc_auto_size:
        radius = min(cross_width * .20, mm_to_bu(context, 4), max_radius)
        length = min(socket_thickness * .45, mm_to_bu(context, 6), max_length)
    else:
        radius = mm_to_bu(context, scene.jc_diameter_mm) * .5
        length = mm_to_bu(context, scene.jc_length_mm)
        if radius > max_radius or length > max_length:
            raise ValueError("Joint does not fit: maximum diameter %.2f mm, length %.2f mm" %
                             (max(0, max_radius * 2) / mm_to_bu(context, 1),
                              max(0, max_length) / mm_to_bu(context, 1)))
    embed = min(length * .6, pin_thickness - wall)
    if radius <= 0 or length <= clearance or embed <= 0:
        raise ValueError("Parts are too small for these wall/clearance values")
    if scene.jc_auto_size and radius < mm_to_bu(context, 1):
        raise ValueError("Parts are too small for automatic sizing; use custom dimensions")
    center = [0.] * 3
    center[axis] = face
    for index, (lo, hi) in zip([i for i in range(3) if i != axis], overlaps):
        center[index] = (lo + hi) * .5
    return dict(center=center, axis=axis, sign=sign, embed=embed, length=length,
                radius=radius, clearance=clearance, tolerance=tol)


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
        try:
            params = joint_parameters(context)
            apply_joint(context, pin, socket, **params)
            scene.jc_preview = False
            self.report({'INFO'}, "Joint created: diameter %.2f mm, length %.2f mm" %
                        (params['radius'] * 2 / mm_to_bu(context, 1),
                         params['length'] / mm_to_bu(context, 1)))
            return {'FINISHED'}
        except Exception as exc:
            self.report({'ERROR'}, "JointCraft: " + str(exc))
            return {'CANCELLED'}


class JOINTCRAFT_PT_main(bpy.types.Panel):
    bl_label = "JointCraft 3D v0.2.2"
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
        layout.prop(scene, "jc_auto_size")
        dimensions = layout.column()
        dimensions.enabled = not scene.jc_auto_size
        dimensions.prop(scene, "jc_diameter_mm")
        dimensions.prop(scene, "jc_length_mm")
        layout.prop(scene, "jc_clearance_mm")
        layout.prop(scene, "jc_wall_mm")
        advanced = layout.box()
        advanced.prop(scene, "jc_show_advanced", text="Advanced", icon='PREFERENCES')
        if scene.jc_show_advanced:
            advanced.prop(scene, "jc_tolerance_mm")
        layout.prop(scene, "jc_preview", toggle=True, icon='HIDE_OFF')
        valid = True
        if scene.jc_preview:
            layout.label(text="Blue: pin / Orange: socket", icon='INFO')
            layout.label(text="X-ray guide; meshes unchanged")
            try:
                params = joint_parameters(context, refresh=False)
                layout.label(text="Diameter %.2f / length %.2f mm" %
                             (params['radius'] * 2 / mm_to_bu(context, 1),
                              params['length'] / mm_to_bu(context, 1)))
            except (ValueError, RuntimeError, ReferenceError) as exc:
                valid = False
                box = layout.box()
                box.alert = True
                # Keep long diagnostics in the operator report; fit the panel.
                import textwrap
                for line in textwrap.wrap(str(exc).split('. Pin ')[0], width=35):
                    box.label(text=line)
        row = layout.row()
        row.enabled = valid
        row.operator("jointcraft.create_auto", text="Create Pin + Socket", icon='MOD_BOOLEAN')
        layout.label(text="Works best with closed mesh cubes")


classes = (JOINTCRAFT_OT_create, JOINTCRAFT_PT_main)


def mesh_in_scene(scene, obj):
    return obj.type == 'MESH' and scene.objects.get(obj.name) == obj


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    from . import preview
    redraw = preview.tag_redraw
    bpy.types.Scene.jc_pin = PointerProperty(name="Pin object", type=bpy.types.Object, poll=mesh_in_scene, update=redraw)
    bpy.types.Scene.jc_socket = PointerProperty(name="Socket object", type=bpy.types.Object, poll=mesh_in_scene, update=redraw)
    bpy.types.Scene.jc_auto_size = BoolProperty(name="Automatic dimensions", default=True, update=redraw)
    bpy.types.Scene.jc_diameter_mm = FloatProperty(name="Pin diameter (mm)", default=8, min=.1, max=100000, precision=2, update=redraw)
    bpy.types.Scene.jc_length_mm = FloatProperty(name="Pin length (mm)", default=6, min=.1, max=100000, precision=2, update=redraw)
    bpy.types.Scene.jc_preview = BoolProperty(name="Show live preview", default=False, update=redraw)
    bpy.types.Scene.jc_clearance_mm = FloatProperty(name="Radial clearance (mm)", default=0.20, min=0.0, max=2.0, precision=2, update=redraw)
    bpy.types.Scene.jc_wall_mm = FloatProperty(name="Minimum wall (mm)", default=1.2, min=0.1, max=20.0, precision=2, update=redraw)
    bpy.types.Scene.jc_tolerance_mm = FloatProperty(name="Face tolerance (mm)", default=0.05, min=0.001, max=2.0, precision=3, update=redraw)
    bpy.types.Scene.jc_show_advanced = BoolProperty(name="Advanced", default=False)
    preview.register()


def unregister():
    from . import preview
    preview.unregister()
    for name in ("jc_pin", "jc_socket", "jc_clearance_mm", "jc_wall_mm", "jc_tolerance_mm", "jc_show_advanced",
                 "jc_auto_size", "jc_diameter_mm", "jc_length_mm", "jc_preview"):
        delattr(bpy.types.Scene, name)
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
