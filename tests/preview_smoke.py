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
HINGE = '--hinge' in sys.argv
PREFIX = 'hinge-' if HINGE else ''

if bpy.app.background:
    import subprocess
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = 0
    result = subprocess.run(
        [bpy.app.binary_path, '--factory-startup', '--window-geometry', '0', '0', '1100', '800',
         '--python-exit-code', '1', '--python', str(Path(__file__).resolve()),
         '--', *(['--hinge'] if HINGE else [])],
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
    if HINGE:
        scene.jc_joint_type = 'HINGE'
        scene.jc_hinge_a, scene.jc_hinge_b = scene.jc_pin, scene.jc_socket
        scene.jc_hinge_center = (.01, 0, 0)
        for obj in (scene.jc_hinge_a, scene.jc_hinge_b):
            obj.scale.z = .4
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
            if scene.jc_hinge_preview if HINGE else scene.jc_preview:
                draws.append(scene.jc_hinge_pin_mm if HINGE else scene.jc_diameter_mm)
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
                if HINGE:
                    scene.jc_hinge_preview = True
                    scene.jc_hinge_pin_mm = 3
                else:
                    scene.jc_preview = True
                    scene.jc_diameter_mm = 8
                    scene.jc_length_mm = 6
            elif stage[0] == 1:
                bpy.ops.screen.screenshot(filepath=str(OUT / (PREFIX + 'preview-small.png')))
                if HINGE:
                    scene.jc_hinge_pin_mm = 4
                    scene.jc_hinge_height_mm = 10
                else:
                    scene.jc_diameter_mm = 14
                    scene.jc_length_mm = 12
            elif stage[0] == 2:
                bpy.ops.screen.screenshot(filepath=str(OUT / (PREFIX + 'preview-large.png')))
                assert all(value in draws for value in ((3, 4) if HINGE else (8, 14))), draws
                assert not errors, errors
                assert (scene.jc_pin.data, scene.jc_socket.data) == original_meshes
                if HINGE:
                    scene.jc_hinge_preview = False
                else:
                    scene.jc_preview = False
            elif stage[0] == 3:
                bpy.ops.screen.screenshot(filepath=str(OUT / (PREFIX + 'preview-off.png')))
                if HINGE:
                    region = next(r for r in area.regions if r.type == 'WINDOW')
                    with bpy.context.temp_override(area=area, region=region):
                        bpy.ops.ed.undo_push(message='Before hinge')
                        bpy.ops.jointcraft.create_hinge()
                        bpy.ops.ed.undo_push(message='After hinge')
                        bpy.ops.ed.undo()
                    assert len(bpy.data.objects) == 2
                    assert all(len(obj.data.vertices) == 8 for obj in bpy.data.objects)
                jc.unregister()
                assert preview._handle is None
                (OUT / (PREFIX + 'preview-smoke-ok.txt')).write_text('GPU preview, parameter updates, off, unregister passed; hinge Undo passed' if HINGE else 'Fixed preview passed', encoding='utf-8')
                # This isolated test must not overwrite the user's quit.blend recovery.
                import os
                sys.stdout.flush()
                sys.stderr.flush()
                os._exit(0)
            stage[0] += 1
            area.tag_redraw()
            return 1.5
        except Exception:
            import traceback
            (OUT / 'preview-smoke-error.txt').write_text(traceback.format_exc(), encoding='utf-8')
            import os
            os._exit(1)

    bpy.app.timers.register(tick, first_interval=2)
