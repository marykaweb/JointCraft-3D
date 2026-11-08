"""GPU smoke test in an isolated hidden Blender window (Windows).

Run with blender -b --factory-startup --python-exit-code 1 --python tests/preview_smoke.py.
The child uses factory settings, saves screenshots under artifacts, then exits.
"""
from pathlib import Path
import sys
import bpy

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'artifacts'
OUT.mkdir(exist_ok=True)

if bpy.app.background:
    import subprocess
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = 0
    result = subprocess.run(
        [bpy.app.binary_path, '--factory-startup', '--window-geometry', '0', '0', '1100', '800',
         '--python-exit-code', '1', '--python', str(Path(__file__).resolve())],
        startupinfo=startup, capture_output=True, text=True, timeout=60)
    (OUT / 'preview-smoke.log').write_text(result.stdout + result.stderr, encoding='utf-8')
    print(result.stdout, result.stderr)
    if result.returncode:
        raise RuntimeError('Preview GPU child failed')
else:
    from mathutils import Vector
    bpy.context.preferences.view.show_splash = False
    sys.path.insert(0, str(ROOT))
    import jointcraft as jc
    from jointcraft import preview
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    jc.register()
    scene = bpy.context.scene
    bpy.ops.mesh.primitive_cube_add(size=.02)
    scene.jc_pin = bpy.context.object
    bpy.ops.mesh.primitive_cube_add(size=.02, location=(.02, 0, 0))
    scene.jc_socket = bpy.context.object
    scene.jc_auto_size = False
    bpy.context.view_layer.update()
    area = next(area for area in bpy.context.screen.areas if area.type == 'VIEW_3D')
    space = area.spaces.active
    space.region_3d.view_location = (.01, 0, 0)
    space.region_3d.view_distance = .075
    space.region_3d.view_rotation = Vector((1, -1.5, 1.1)).to_track_quat('Z', 'Y')
    space.clip_start = .0001
    space.clip_end = 10
    space.overlay.show_floor = False
    space.overlay.show_axis_x = False
    space.overlay.show_axis_y = False
    space.show_region_ui = True
    space.shading.type = 'SOLID'
    # Turn draw callback failures into a failing test, rather than console noise.
    errors, draws = [], []
    original_draw = preview.draw

    def checked_draw():
        try:
            original_draw()
            if scene.jc_preview:
                draws.append(scene.jc_diameter_mm)
        except Exception as exc:
            errors.append(repr(exc))

    preview.unregister()
    preview.draw = checked_draw
    preview.register()
    original_meshes = (scene.jc_pin.data, scene.jc_socket.data)
    stage = [0]

    def tick():
        try:
            if stage[0] == 0:
                scene.jc_preview = True
                scene.jc_diameter_mm = 8
                scene.jc_length_mm = 6
            elif stage[0] == 1:
                bpy.ops.screen.screenshot(filepath=str(OUT / 'preview-8mm.png'))
                scene.jc_diameter_mm = 14
                scene.jc_length_mm = 12
            elif stage[0] == 2:
                bpy.ops.screen.screenshot(filepath=str(OUT / 'preview-14mm.png'))
                assert 8 in draws and 14 in draws, draws
                assert not errors, errors
                assert (scene.jc_pin.data, scene.jc_socket.data) == original_meshes
                scene.jc_preview = False
            elif stage[0] == 3:
                bpy.ops.screen.screenshot(filepath=str(OUT / 'preview-off.png'))
                jc.unregister()
                assert preview._handle is None
                (OUT / 'preview-smoke-ok.txt').write_text('GPU preview passed: 8mm, 14mm, off, unregister', encoding='utf-8')
                bpy.ops.wm.quit_blender()
                return None
            stage[0] += 1
            area.tag_redraw()
            return 1.5
        except Exception:
            import traceback
            (OUT / 'preview-smoke-error.txt').write_text(traceback.format_exc(), encoding='utf-8')
            import os
            os._exit(1)

    bpy.app.timers.register(tick, first_interval=2)
