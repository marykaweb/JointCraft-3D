"""Non-destructive, x-ray viewport guides. No scene objects or modifiers."""
import math
import bpy

_handle = None


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


def draw():
    context = bpy.context
    scene = context.scene
    if (context.mode != 'OBJECT' or not scene or not getattr(scene, 'jc_preview', False)
            or not context.region_data):
        return
    # Respect local view: never draw guides for objects absent from this view.
    for obj in (scene.jc_pin, scene.jc_socket):
        if obj is None or not obj.visible_get(view_layer=context.view_layer,
                                              viewport=context.space_data):
            return
    from . import joint_parameters
    try:
        params = joint_parameters(context, refresh=False)
        pin, socket = preview_lines(params)
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
        for coords, color, width in ((socket, (1., .35, .04, .9), 2.),
                                     (pin, (.05, .65, 1., 1.), 2.5)):
            shader.uniform_float('lineWidth', width)
            shader.uniform_float('color', color)
            batch_for_shader(shader, 'LINES', {'pos': coords}).draw(shader)
    finally:
        gpu.state.depth_test_set(old_depth)
        gpu.state.blend_set(old_blend)


def register():
    global _handle
    if _handle is None and not bpy.app.background:
        _handle = bpy.types.SpaceView3D.draw_handler_add(draw, (), 'WINDOW', 'POST_VIEW')


def unregister():
    global _handle
    if _handle is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_handle, 'WINDOW')
        _handle = None
    tag_redraw()
