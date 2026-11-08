"""Non-destructive, x-ray viewport guides. No scene objects or modifiers."""
import math
import bpy

_handle = None
_fixed_parameters = None


def tag_redraw(_scene=None, context=None):
    context = context or bpy.context
    if context.window_manager:
        for window in context.window_manager.windows:
            for area in window.screen.areas:
                if area.type == 'VIEW_3D':
                    area.tag_redraw()


def cylinder_lines(center, axis, sign, start, end, radius, segments=48):
    """World-space ring edges and four rails, matching cylinder_mesh."""
    others = [i for i in range(3) if i != axis]
    rings = []
    for t in (start, end):
        ring = []
        for j in range(segments):
            point = list(center)
            point[axis] += sign * t
            angle = math.tau * j / segments
            point[others[0]] += radius * math.cos(angle)
            point[others[1]] += radius * math.sin(angle)
            ring.append(tuple(point))
        rings.append(ring)
    lines = []
    for ring in rings:
        for j in range(segments):
            lines.extend((ring[j], ring[(j + 1) % segments]))
    for j in range(0, segments, segments // 4):
        lines.extend((rings[0][j], rings[1][j]))
    return lines


def preview_lines(params):
    base = (params['center'], params['axis'], params['sign'])
    pin = cylinder_lines(*base, -params['embed'], params['length'], params['radius'])
    socket = cylinder_lines(*base, -max(params['tolerance'], params['clearance'] * .15),
                            params['length'] + params['clearance'],
                            params['radius'] + params['clearance'])
    return pin, socket


def hinge_lines(p):
    from . import hinge_geometry as geo
    groups = [[], [], []]

    def add(surface, group, circular=False):
        vertices, faces = geo.world_surface(surface, p['center'], p['u'], p['v'], p['w'])
        edges = set()
        for face in faces:
            for j, first in enumerate(face):
                second = face[(j + 1) % len(face)]
                if circular and abs(first - second) == 64 and first % 16:
                    continue
                edges.add(tuple(sorted((first, second))))
        for a, b in sorted(edges):
            groups[group].extend((vertices[a], vertices[b]))

    for i, (start, end) in enumerate(p['intervals']):
        group = 1 if i == 1 else 0
        add(geo.cylinder(p['outer'], start, end), group, True)
        add(geo.cylinder(p['hole'], start, end), group, True)
        add(geo.support(p, 1 if i == 1 else -1, start, end), group)
    add(geo.pin(p), 2, True)
    # Dashed outlines show where the original mating faces will be bevelled.
    for side, vlo, vhi, zlo, zhi in p.get('cut_guides', []):
        group = 0 if side < 0 else 1
        points = []
        for y in (vlo, 0, vhi):
            x = side * (p['axial'] / 2 + math.tan(math.radians(p['angle'] / 2)) * abs(y))
            points.append((x, y))
        for z in (zlo, zhi):
            surface = ([(x, y, z) for x, y in points], [(0, 1, 2)])
            vertices, _ = geo.world_surface(surface, p['center'], p['u'], p['v'], p['w'])
            for start, end in zip(vertices, vertices[1:]):
                for j in range(0, 12, 2):
                    groups[group].extend(tuple(start[i] + (end[i] - start[i]) * t / 12 for i in range(3))
                                         for t in (j, j + 1))
    return groups


def draw():
    context = bpy.context
    scene = context.scene
    if context.mode != 'OBJECT' or not scene or not context.region_data:
        return
    hinge = getattr(scene, 'jc_joint_type', 'FIXED') == 'HINGE'
    if not (scene.jc_hinge_preview if hinge else scene.jc_preview):
        return
    # Respect local view: never draw guides for objects absent from this view.
    objects = (scene.jc_hinge_a, scene.jc_hinge_b) if hinge else (scene.jc_pin, scene.jc_socket)
    for obj in objects:
        if obj is None or not obj.visible_get(view_layer=context.view_layer,
                                              viewport=context.space_data):
            return
    try:
        if hinge:
            from . import hinges
            params = hinges.parameters(context, refresh=False)
            a, b, pin = hinge_lines(params)
            batches = ((a, (.05, .65, 1., 1.), 2.),
                       (b, (1., .35, .04, .9), 2.), (pin, (.2, 1., .25, 1.), 2.))
        else:
            if _fixed_parameters is None:
                return
            params = _fixed_parameters(context, refresh=False)
            pin, socket = preview_lines(params)
            batches = ((socket, (1., .35, .04, .9), 2.), (pin, (.05, .65, 1., 1.), 2.5))
    except (ValueError, RuntimeError, ReferenceError):
        return
    import gpu
    from gpu_extras.batch import batch_for_shader
    shader = gpu.shader.from_builtin('POLYLINE_UNIFORM_COLOR')
    old_depth = gpu.state.depth_test_get()
    old_blend = gpu.state.blend_get()
    try:
        # The joint lies inside touching meshes; x-ray makes it visible there.
        gpu.state.depth_test_set('NONE')
        gpu.state.blend_set('ALPHA')
        shader.bind()
        shader.uniform_float('viewportSize', gpu.state.viewport_get()[2:])
        for coords, color, width in batches:
            shader.uniform_float('lineWidth', width)
            shader.uniform_float('color', color)
            batch_for_shader(shader, 'LINES', {'pos': coords}).draw(shader)
    finally:
        gpu.state.depth_test_set(old_depth)
        gpu.state.blend_set(old_blend)


def register(fixed_parameters=None):
    global _handle, _fixed_parameters
    if fixed_parameters is not None:
        _fixed_parameters = fixed_parameters
    if _handle is None and not bpy.app.background:
        _handle = bpy.types.SpaceView3D.draw_handler_add(draw, (), 'WINDOW', 'POST_VIEW')


def unregister():
    global _handle
    if _handle is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_handle, 'WINDOW')
        _handle = None
    tag_redraw()
