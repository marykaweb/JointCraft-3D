"""Staged three-knuckle hinge construction and sampled motion validation."""
import math
import bpy
import bmesh
from mathutils import Vector, Matrix
from bpy.props import BoolProperty, EnumProperty, FloatProperty, FloatVectorProperty, PointerProperty
from . import hinge_geometry as geo
from .utils import mm_to_bu, bounds_world, boolean_mesh


def placement(context, refresh=True):
    s = context.scene
    if refresh:
        context.view_layer.update()
    a, b = s.jc_hinge_a, s.jc_hinge_b
    if a is None or b is None or a == b:
        raise ValueError('Choose two different mesh objects')
    for obj in (a, b):
        if obj.type != 'MESH' or not obj.data.vertices:
            raise ValueError('Both parts must be non-empty meshes')
        if s.objects.get(obj.name) != obj or context.view_layer.objects.get(obj.name) != obj:
            raise ValueError("Object '%s' is not in the active scene/view layer" % obj.name)
        if obj.library or obj.data.library or obj.override_library or not obj.is_editable:
            raise ValueError('Linked and library override objects cannot be edited')
        if obj.modifiers or obj.data.shape_keys:
            raise ValueError('Apply modifiers and resolve shape keys first')
        if abs(obj.matrix_world.determinant()) < 1e-12:
            raise ValueError('Object transform must be invertible')
    center_obj = s.jc_hinge_center_object
    if center_obj:
        if s.objects.get(center_obj.name) != center_obj:
            raise ValueError('Center Empty is not in the active scene')
        center = center_obj.matrix_world.translation.copy()
    else:
        # FloatVectorProperty length coordinates follow Blender's unit display.
        center = Vector(s.jc_hinge_center)
    axis = 'XYZ'.index(s.jc_hinge_axis)
    w = Vector(tuple(1 if i == axis else 0 for i in range(3)))
    basis = Vector((0, 1, 0) if axis == 0 else (1, 0, 0))
    u = Matrix.Rotation(math.radians(s.jc_hinge_direction), 3, w) @ basis
    v = w.cross(u)
    p = dict(center=center, u=u, v=v, w=w, axis=axis)
    corners = [obj.matrix_world @ Vector(corner) for obj in (a, b) for corner in obj.bound_box]
    p['extent_base'] = max((point - center).length for point in corners) * 2
    p['cut_guides'] = []
    p['local_bounds'], p['part_sizes'] = [], []
    # Side selection is explicit: A on -u, B on +u. No guessing from origins.
    for obj, side in ((a, -1), (b, 1)):
        low, high = bounds_world(obj)
        midpoint = (Vector(low) + Vector(high)) * .5
        if side * (midpoint - center).dot(u) <= mm_to_bu(context, s.jc_hinge_wall_mm) * .1:
            raise ValueError('Check center/direction: Part A must be on -direction, Part B on +direction')
        local_corners = [obj.matrix_world @ Vector(corner) - center for corner in obj.bound_box]
        p['local_bounds'].append(tuple((min(c.dot(basis) for c in local_corners),
                                       max(c.dot(basis) for c in local_corners))
                                      for basis in (u, v, w)))
        p['part_sizes'].append([high[i] - low[i] for i in range(3)])
        p['cut_guides'].append((side, min(c.dot(v) for c in local_corners), max(c.dot(v) for c in local_corners),
                                min(c.dot(w) for c in local_corners), max(c.dot(w) for c in local_corners)))
    return p


def parameters(context, refresh=True):
    s = context.scene
    frame = placement(context, refresh)
    bu = lambda val: mm_to_bu(context, val)
    p = geo.dimensions(bu(s.jc_hinge_pin_mm), bu(s.jc_hinge_radial_mm),
                       bu(s.jc_hinge_axial_mm), bu(s.jc_hinge_height_mm),
                       bu(s.jc_hinge_wall_mm), bu(s.jc_hinge_outer_mm),
                       bu(s.jc_hinge_middle_mm), bu(s.jc_hinge_pin_length_mm),
                       s.jc_hinge_angle)
    p.update(frame)
    p['extent'] = p['extent_base'] + p['reach'] * 4
    for index, (obj, bounds) in enumerate(zip((s.jc_hinge_a, s.jc_hinge_b), p['local_bounds'])):
        bands = (p['intervals'][0], p['intervals'][2]) if index == 0 else (p['intervals'][1],)
        for start, end in bands:
            if min(end, bounds[2][1]) <= max(start, bounds[2][0]):
                raise ValueError("%s '%s': knuckle is outside the part thickness along %s. "
                                 "Move Center into the thickness or reduce hinge height (current %.2f mm)." %
                                 ('Part A' if index == 0 else 'Part B', obj.name,
                                  s.jc_hinge_axis, s.jc_hinge_height_mm))
    return p


def fitted_dimensions(context):
    s = context.scene
    p = placement(context)
    bounds = p['local_bounds']
    available = 2 * min(-max(b[2][0] for b in bounds), min(b[2][1] for b in bounds))
    depth = min(-bounds[0][0][0], bounds[1][0][1])
    half_width = min(min(-b[1][0], b[1][1]) for b in bounds)
    bu = lambda val: mm_to_bu(context, val)
    suggestion = geo.fit_dimensions(available, depth, half_width,
                                    bu(s.jc_hinge_wall_mm), bu(s.jc_hinge_radial_mm),
                                    bu(s.jc_hinge_axial_mm), s.jc_hinge_fit_percent)
    if suggestion['pin_diameter'] < bu(.1):
        raise ValueError('Suggested pin is smaller than 0.1 mm; set model dimensions first')
    return {key: value / bu(1) for key, value in suggestion.items()}


def mesh_info(mesh, matrix=None, require_connected=True):
    bm = bmesh.new()
    try:
        bm.from_mesh(mesh)
        if matrix is not None:
            bm.transform(matrix)
        if not bm.faces or any(not e.is_manifold for e in bm.edges):
            raise ValueError('Result is not a closed manifold mesh')
        if any(f.calc_area() <= 1e-18 for f in bm.faces):
            raise ValueError('Result contains degenerate faces')
        if require_connected:
            bm.verts.ensure_lookup_table()
            seen, stack = set(), [bm.verts[0]]
            while stack:
                vert = stack.pop()
                if vert in seen:
                    continue
                seen.add(vert)
                stack.extend(e.other_vert(vert) for e in vert.link_edges)
            if len(seen) != len(bm.verts):
                raise ValueError('Disconnected geometry: move center closer or enlarge attachment reach')
        return abs(bm.calc_volume(signed=True))
    finally:
        bm.free()


class Stage:
    def __init__(self, context):
        self.context, self.objects, self.meshes = context, [], []
        self.tolerance = 1e-8

    def object(self, mesh, matrix=None):
        self.meshes.append(mesh)
        obj = bpy.data.objects.new('JC_Hinge_Temp', mesh)
        self.objects.append(obj)
        self.context.scene.collection.objects.link(obj)
        if matrix is not None:
            obj.matrix_world = matrix
        return obj

    def surface(self, p, surface):
        vertices, faces = geo.world_surface(surface, p['center'], p['u'], p['v'], p['w'])
        mesh = bpy.data.meshes.new('JC_Hinge_Temp')
        self.meshes.append(mesh)
        mesh.from_pydata(vertices, [], faces)
        mesh.update()
        return self.object(mesh)

    def boolean(self, target, cutter, operation):
        result = boolean_mesh(self.context, target, cutter, operation)
        self.meshes.append(result)
        bm = bmesh.new()
        try:
            bm.from_mesh(result)
            bm.transform(target.matrix_world)
            bmesh.ops.remove_doubles(bm, verts=list(bm.verts), dist=self.tolerance)
            bmesh.ops.dissolve_degenerate(bm, edges=list(bm.edges), dist=self.tolerance)
            bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
            bm.transform(target.matrix_world.inverted())
            bm.to_mesh(result)
            result.update()
        finally:
            bm.free()
        target.data = result

    def overlap(self, a, b):
        # Empty intersections are a valid zero-volume result, unlike build steps.
        mod = a.modifiers.new('JC collision probe', 'BOOLEAN')
        mod.operation, mod.solver, mod.object = 'INTERSECT', 'EXACT', b
        mesh = None
        try:
            depsgraph = self.context.evaluated_depsgraph_get()
            mesh = bpy.data.meshes.new_from_object(a.evaluated_get(depsgraph), depsgraph=depsgraph)
            if mesh is None:
                raise RuntimeError('Collision evaluation failed')
            if not mesh.polygons:
                return 0.
            bm = bmesh.new()
            try:
                bm.from_mesh(mesh)
                bm.transform(a.matrix_world)
                return abs(bm.calc_volume(signed=True))
            finally:
                bm.free()
        finally:
            a.modifiers.remove(mod)
            if mesh is not None:
                bpy.data.meshes.remove(mesh)

    def close(self):
        for obj in self.objects:
            bpy.data.objects.remove(obj, do_unlink=True)
        for mesh in set(self.meshes):
            if mesh.users == 0:
                bpy.data.meshes.remove(mesh)


def create_pin_object(context, mesh):
    obj = bpy.data.objects.new('Hinge Pin', mesh)
    try:
        context.scene.collection.objects.link(obj)
        return obj
    except Exception:
        bpy.data.objects.remove(obj, do_unlink=True)
        raise


def assign_mesh(obj, mesh):
    obj.data = mesh


def build(context, p):
    """Prepare everything on isolated copies; commit all three objects together."""
    s = context.scene
    originals = (s.jc_hinge_a, s.jc_hinge_b)
    old_meshes = tuple(obj.data for obj in originals)
    st = Stage(context)
    st.tolerance = min(p['wall'], p['axial'], p['hole'] - p['pin_radius']) * .0001
    final_pin = None
    committed = False
    try:
        parts = [st.object(obj.data.copy(), obj.matrix_world.copy()) for obj in originals]
        for obj in parts:
            mesh_info(obj.data, obj.matrix_world)
        clearance = st.surface(p, geo.cylinder(p['relief_radius'], -p['extent'], p['extent']))
        bore = st.surface(p, geo.cylinder(p['hole_cut_radius'], -p['extent'], p['extent']))
        for index, (part, side) in enumerate(zip(parts, (-1, 1))):
            st.boolean(part, st.surface(p, geo.keeper(p, side, p['extent'])), 'INTERSECT')
            st.boolean(part, clearance, 'DIFFERENCE')
            bands = (p['intervals'][0], p['intervals'][2]) if index == 0 else (p['intervals'][1],)
            for start, end in bands:
                support = st.surface(p, geo.support(p, side, start, end))
                overlap = st.overlap(part, support)
                if overlap < p['wall'] ** 3 * .1:
                    raise ValueError("%s '%s': insufficient attachment material for knuckle %.2f..%.2f mm along %s. "
                                     "Check Center and attachment direction; support reach is %.2f mm." %
                                     ('Part A' if index == 0 else 'Part B', originals[index].name,
                                      start / mm_to_bu(context, 1), end / mm_to_bu(context, 1),
                                      s.jc_hinge_axis, p['reach'] / mm_to_bu(context, 1)))
                st.boolean(part, support, 'UNION')
                st.boolean(part, st.surface(p, geo.cylinder(p['outer'], start, end)), 'UNION')
            st.boolean(part, bore, 'DIFFERENCE')
            mesh_info(part.data, part.matrix_world)
        pin = st.surface(p, geo.pin(p))
        mesh_info(pin.data)
        threshold = min(p['wall'], p['axial'], p['hole'] - p['pin_radius']) ** 3 * .001
        for part in parts:
            if st.overlap(part, pin) > threshold:
                raise ValueError('Pin insertion corridor is obstructed')
        center = p['center']
        matrix = parts[1].matrix_world.copy()
        samples = {0., -p['angle'], p['angle']}
        samples.update(i for i in range(-int(p['angle']), int(p['angle']) + 1, 5))
        for degrees in sorted(samples):
            rotation = Matrix.Translation(center) @ Matrix.Rotation(math.radians(degrees), 4, p['w']) @ Matrix.Translation(-center)
            parts[1].matrix_world = rotation @ matrix
            if st.overlap(parts[0], parts[1]) > threshold:
                raise ValueError('Parts collide near %d degrees; move center or reduce rotation range' % degrees)
            if st.overlap(parts[1], pin) > threshold:
                raise ValueError('Middle knuckle intersects pin near %d degrees' % degrees)
        parts[1].matrix_world = matrix
        for source, result in zip(originals, parts):
            result.data.name = source.data.name + ' Hinge'
        pin.data.name = 'Hinge Pin Mesh'
        # Create and link pin before committing source meshes, so link failures roll back.
        final_pin = create_pin_object(context, pin.data)
        final_pin['jointcraft_role'] = 'hinge_pin'
        final_pin['assembly_note'] = 'Insert from head side. Fix exposed tip to Part A only; keep glue away from Part B.'
        final_pin['jointcraft_center'] = list(p['center'])
        final_pin['jointcraft_axis'] = list(p['w'])
        try:
            assign_mesh(originals[0], parts[0].data)
            assign_mesh(originals[1], parts[1].data)
        except Exception:
            originals[0].data, originals[1].data = old_meshes
            raise
        committed = True
        return final_pin
    finally:
        if not committed and final_pin is not None:
            bpy.data.objects.remove(final_pin, do_unlink=True)
        st.close()
        if committed:
            for mesh in set(old_meshes):
                if mesh.users == 0:
                    bpy.data.meshes.remove(mesh)


class JOINTCRAFT_OT_hinge(bpy.types.Operator):
    bl_idname = 'jointcraft.create_hinge'
    bl_label = 'Create Articulated Hinge'
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'OBJECT'

    def execute(self, context):
        try:
            p = parameters(context)
            build(context, p)
            context.scene.jc_hinge_preview = False
            self.report({'INFO'}, 'Hinge created; sampled swing +/-%.0f degrees. Secure pin tip to Part A only.' % p['angle'])
            return {'FINISHED'}
        except Exception as exc:
            self.report({'ERROR'}, 'JointCraft hinge: ' + str(exc))
            return {'CANCELLED'}


class JOINTCRAFT_OT_hinge_cursor(bpy.types.Operator):
    bl_idname = 'jointcraft.hinge_cursor'
    bl_label = 'Use 3D Cursor'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        context.scene.jc_hinge_center_object = None
        context.scene.jc_hinge_center = context.scene.cursor.location
        return {'FINISHED'}


class JOINTCRAFT_OT_hinge_fit(bpy.types.Operator):
    bl_idname = 'jointcraft.hinge_fit'
    bl_label = 'Fit Dimensions to Parts'
    bl_description = 'Suggest proportions from part thickness around Center; keeps clearances and wall, resets advanced dimensions. Use before creating the hinge'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        s = context.scene
        old = tuple(getattr(s, name) for name in FIT_PROPERTIES)
        try:
            fit = fitted_dimensions(context)
            s.jc_hinge_pin_mm = fit['pin_diameter']
            s.jc_hinge_height_mm = fit['height']
            s.jc_hinge_outer_mm = fit['outer_diameter']
            s.jc_hinge_middle_mm = 0
            s.jc_hinge_pin_length_mm = 0
            parameters(context)
            s.jc_hinge_preview = True
            self.report({'INFO'}, 'Suggested height %.2f mm, pin diameter %.2f mm; inspect live preview before creating' %
                        (s.jc_hinge_height_mm, s.jc_hinge_pin_mm))
            return {'FINISHED'}
        except Exception as exc:
            for name, value in zip(FIT_PROPERTIES, old):
                setattr(s, name, value)
            self.report({'ERROR'}, 'JointCraft sizing: ' + str(exc))
            return {'CANCELLED'}


def draw_panel(layout, context):
    s = context.scene
    layout.label(text='Placement: Manual (world coordinates)')
    for prop in ('jc_hinge_a', 'jc_hinge_b', 'jc_hinge_center_object'):
        layout.prop(s, prop)
    model_size = layout.box()
    model_size.label(text='Model dimensions (mm)')
    for label, obj in (('Part A', s.jc_hinge_a), ('Part B', s.jc_hinge_b)):
        if obj is not None and obj.type == 'MESH':
            low, high = bounds_world(obj)
            size = [(high[i] - low[i]) / mm_to_bu(context, 1) for i in range(3)]
            model_size.label(text='%s: %.1f x %.1f x %.1f' % (label, *size))
            if max(size) > 500:
                model_size.label(text='Large model: check final print size', icon='INFO')
    if s.jc_hinge_center_object is None:
        layout.prop(s, 'jc_hinge_center')
        layout.operator('jointcraft.hinge_cursor', icon='PIVOT_CURSOR')
    layout.prop(s, 'jc_hinge_axis')
    layout.prop(s, 'jc_hinge_direction')
    layout.label(text='A: -direction / B: +direction')
    sizing = layout.box()
    sizing.label(text='Size relative to parts')
    sizing.prop(s, 'jc_hinge_fit_percent')
    sizing.operator('jointcraft.hinge_fit', icon='FULLSCREEN_ENTER')
    sizing.label(text='Use before Create; keeps print gaps')
    for prop in ('jc_hinge_pin_mm', 'jc_hinge_radial_mm', 'jc_hinge_axial_mm',
                 'jc_hinge_height_mm', 'jc_hinge_wall_mm', 'jc_hinge_angle'):
        layout.prop(s, prop)
    layout.prop(s, 'jc_hinge_advanced', icon='PREFERENCES')
    if s.jc_hinge_advanced:
        layout.label(text='0 = automatic')
        for prop in ('jc_hinge_outer_mm', 'jc_hinge_middle_mm', 'jc_hinge_pin_length_mm'):
            layout.prop(s, prop)
    layout.prop(s, 'jc_hinge_preview', toggle=True, icon='HIDE_OFF')
    layout.label(text='Blue: A / Orange: B / Green: pin')
    layout.label(text='Cuts mating faces for swing room', icon='MOD_BOOLEAN')
    layout.label(text='Dashed lines: bevel boundaries')
    valid = True
    try:
        p = parameters(context, refresh=False)
        summary = layout.box()
        summary.label(text='Actual dimensions (mm)', icon='DRIVER_DISTANCE')
        unit = mm_to_bu(context, 1)
        summary.label(text='Hinge height: %.2f' % (p['height'] / unit))
        summary.label(text='Outer diameter: %.2f' % (2 * p['outer'] / unit))
        summary.label(text='Pin diameter: %.2f' % (2 * p['pin_radius'] / unit))
        summary.label(text='Pin total length: %.2f' % ((p['shaft_length'] + p['head_thickness']) / unit))
        summary.label(text='Knuckles A/B/A: %.2f / %.2f / %.2f' %
                           (p['ear'] / unit, p['middle'] / unit, p['ear'] / unit))
        thickness = min(b[2][1] - b[2][0] for b in p['local_bounds'])
        summary.label(text='Height / part thickness: %.1f%%' % (100 * p['height'] / thickness))
        if p['height'] < thickness * .05:
            summary.label(text='Very small: try Fit Dimensions', icon='INFO')
    except (ValueError, RuntimeError, ReferenceError) as exc:
        import textwrap
        valid = False
        box = layout.box()
        box.alert = True
        for line in textwrap.wrap(str(exc), 35):
            box.label(text=line)
    row = layout.row()
    row.enabled = valid
    row.operator('jointcraft.create_hinge', icon='MOD_BOOLEAN')
    layout.label(text='Glue pin tip to Part A only', icon='INFO')


FIT_PROPERTIES = ('jc_hinge_pin_mm', 'jc_hinge_height_mm', 'jc_hinge_outer_mm',
                  'jc_hinge_middle_mm', 'jc_hinge_pin_length_mm')
CLASSES = (JOINTCRAFT_OT_hinge, JOINTCRAFT_OT_hinge_cursor, JOINTCRAFT_OT_hinge_fit)
PROPERTIES = ('jc_joint_type', 'jc_hinge_a', 'jc_hinge_b', 'jc_hinge_axis',
              'jc_hinge_center_object', 'jc_hinge_center', 'jc_hinge_direction',
              'jc_hinge_preview', 'jc_hinge_advanced', 'jc_hinge_pin_mm',
              'jc_hinge_radial_mm', 'jc_hinge_axial_mm', 'jc_hinge_height_mm',
              'jc_hinge_wall_mm', 'jc_hinge_outer_mm', 'jc_hinge_middle_mm',
              'jc_hinge_pin_length_mm', 'jc_hinge_angle', 'jc_hinge_fit_percent')


def register(redraw, mesh_in_scene):
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    scene = bpy.types.Scene
    scene.jc_joint_type = EnumProperty(name='Joint type', items=(('FIXED', 'Fixed Pin + Socket', ''), ('HINGE', 'Articulated Hinge', '')), default='FIXED', update=redraw)
    scene.jc_hinge_a = PointerProperty(name='Part A (outer knuckles)', type=bpy.types.Object, poll=mesh_in_scene, update=redraw)
    scene.jc_hinge_b = PointerProperty(name='Part B (middle knuckle)', type=bpy.types.Object, poll=mesh_in_scene, update=redraw)
    scene.jc_hinge_axis = EnumProperty(name='World axis', items=tuple((v, v, '') for v in 'XYZ'), default='Z', update=redraw)
    scene.jc_hinge_center_object = PointerProperty(name='Center Empty', type=bpy.types.Object, poll=lambda s, o: o.type == 'EMPTY' and s.objects.get(o.name) == o, update=redraw)
    scene.jc_hinge_center = FloatVectorProperty(name='Center', subtype='TRANSLATION', unit='LENGTH', update=redraw)
    scene.jc_hinge_direction = FloatProperty(name='Attachment direction (deg)', default=0, min=-180, max=180, update=redraw)
    scene.jc_hinge_angle = FloatProperty(name='Swing each way (deg)', default=30, min=5, max=60, precision=0, update=redraw)
    scene.jc_hinge_preview = BoolProperty(name='Show live preview', default=True, update=redraw)
    scene.jc_hinge_advanced = BoolProperty(name='Advanced dimensions', default=False, update=redraw)
    scene.jc_hinge_fit_percent = FloatProperty(name='Height / thickness (%)', default=80,
                                              min=10, max=100, precision=0, step=1, update=redraw)
    for name, label, default, minimum in (
            ('jc_hinge_pin_mm', 'Pin diameter (mm)', 3, .1),
            ('jc_hinge_radial_mm', 'Radial clearance (mm)', .25, .01),
            ('jc_hinge_axial_mm', 'Axial clearance per gap (mm)', .30, .01),
            ('jc_hinge_height_mm', 'Total hinge height (mm)', 8, .1),
            ('jc_hinge_wall_mm', 'Minimum wall (mm)', 1.2, .1),
            ('jc_hinge_outer_mm', 'Outer diameter (mm)', 0, 0),
            ('jc_hinge_middle_mm', 'Middle knuckle width (mm)', 0, 0),
            ('jc_hinge_pin_length_mm', 'Pin shaft length (mm)', 0, 0)):
        setattr(scene, name, FloatProperty(name=label, default=default, min=minimum,
                                          soft_max=max(default * 4, 20), precision=2, step=1, update=redraw))


def unregister():
    for prop in PROPERTIES:
        delattr(bpy.types.Scene, prop)
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
