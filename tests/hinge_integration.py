"""Blender 5.2 hinge regression tests, including rollback and motion."""
import math
import sys
from pathlib import Path
import unittest
from unittest.mock import patch
import bpy
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import jointcraft as jc
from jointcraft import hinges as h, hinge_geometry as g, preview


class HingeTests(unittest.TestCase):
    def setUp(self):
        bpy.ops.wm.read_factory_settings(use_empty=True)
        self.scene = bpy.context.scene

    def parts(self, axis='Z', scale=1, translation=(0, 0, 0)):
        s = self.scene
        s.unit_settings.scale_length = scale
        axis_index = 'XYZ'.index(axis)
        direction = 1 if axis == 'X' else 0
        center = Vector(translation)
        created = []
        for sign in (-1, 1):
            position = center.copy()
            position[direction] += sign * .01 / scale
            bpy.ops.mesh.primitive_cube_add(size=.02 / scale, location=position)
            obj = bpy.context.object
            obj.scale[axis_index] = .4
            created.append(obj)
        s.jc_hinge_a, s.jc_hinge_b = created
        s.jc_hinge_center = center
        s.jc_hinge_axis = axis
        s.jc_joint_type = 'HINGE'
        bpy.context.view_layer.update()
        return created

    def test_build_all_axes_and_swing(self):
        for axis in 'XYZ':
            with self.subTest(axis=axis):
                self.setUp()
                a, b = self.parts(axis)
                p = h.parameters(bpy.context)
                originals = [obj.matrix_world.copy() for obj in (a, b)]
                self.assertEqual(bpy.ops.jointcraft.create_hinge(), {'FINISHED'})
                self.assertEqual(len(bpy.data.objects), 3)
                self.assertFalse(self.scene.jc_hinge_preview)
                pin = next(obj for obj in bpy.data.objects if obj.get('jointcraft_role') == 'hinge_pin')
                for obj in (a, b, pin):
                    self.assertGreater(h.mesh_info(obj.data, obj.matrix_world), 0)
                    self.assertFalse(obj.modifiers)
                bpy.context.view_layer.update()
                # Probe the actual bores from the empty axis toward the ring.
                for obj, band in ((a, p['intervals'][0]), (b, p['intervals'][1]),
                                  (a, p['intervals'][2])):
                    origin = p['center'] + p['w'] * ((band[0] + band[1]) / 2)
                    inverse = obj.matrix_world.inverted()
                    hit, point, _normal, _face = obj.ray_cast(
                        inverse @ origin, inverse.to_3x3() @ p['u'])
                    self.assertTrue(hit)
                    self.assertAlmostEqual((obj.matrix_world @ point - origin).length,
                                           p['hole_cut_radius'], delta=1e-6)
                self.assertEqual([a.matrix_world, b.matrix_world], originals)
                stage = h.Stage(bpy.context)
                try:
                    for degrees in range(-30, 31, 2):
                        b.matrix_world = Matrix.Translation(p['center']) @ Matrix.Rotation(math.radians(degrees), 4, p['w']) @ Matrix.Translation(-p['center']) @ originals[1]
                        self.assertLess(stage.overlap(a, b), 1e-13)
                    b.matrix_world = originals[1]
                    for obj in (a, b):
                        self.assertLess(stage.overlap(obj, pin), 1e-13)
                finally:
                    b.matrix_world = originals[1]
                    stage.close()
                self.assertFalse(any(obj.name.startswith('JC_') for obj in bpy.data.objects))
                self.assertFalse(any(mesh.name.startswith('JC_') for mesh in bpy.data.meshes))
                self.assertTrue(all(mesh.users for mesh in bpy.data.meshes))

    def test_units_transforms_and_shared_mesh(self):
        a, b = self.parts(scale=.001, translation=(10, 20, 30))
        b.data = a.data
        # A third user of this datablock must be preserved.
        copy = bpy.data.objects.new('Keep original', a.data)
        original = copy.data
        self.scene.collection.objects.link(copy)
        p = h.parameters(bpy.context)
        self.assertAlmostEqual(p['hole'], 1.75, places=6)
        h.build(bpy.context, p)
        self.assertEqual(copy.data, original)
        self.assertNotEqual(a.data, b.data)
        self.assertEqual(len(original.vertices), 8)

    def test_preview_is_non_destructive(self):
        self.parts()
        state = (set(bpy.data.objects), set(bpy.data.meshes))
        p = h.parameters(bpy.context)
        lines = preview.hinge_lines(p)
        self.assertTrue(all(lines))
        self.scene.jc_hinge_pin_mm = 4
        changed = preview.hinge_lines(h.parameters(bpy.context))
        self.assertNotEqual(lines, changed)
        self.assertEqual((set(bpy.data.objects), set(bpy.data.meshes)), state)
        self.assertAlmostEqual(p['intervals'][1][0] - p['intervals'][0][1], .0003)
        self.assertAlmostEqual(p['hole'] - p['pin_radius'], .00025)

    def test_invalid_dimensions(self):
        for kwargs in ({'pin_diameter': 0}, {'radial': 0}, {'height': .001},
                       {'outer_diameter': .003}, {'pin_length': .002}):
            args = dict(pin_diameter=.003, radial=.00025, axial=.0003,
                        height=.008, wall=.0012)
            args.update(kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                g.dimensions(**args)

    def test_bad_selection_and_off_scene(self):
        a, b = self.parts()
        self.scene.jc_hinge_b = a
        with self.assertRaisesRegex(ValueError, 'different'):
            h.parameters(bpy.context)
        self.scene.jc_hinge_b = b
        self.scene.collection.objects.unlink(b)
        with self.assertRaisesRegex(ValueError, 'active scene'):
            h.parameters(bpy.context)

    def test_empty_center_and_cursor(self):
        self.parts()
        bpy.ops.object.empty_add(location=(0, 0, 0))
        self.scene.jc_hinge_center_object = bpy.context.object
        self.assertEqual(tuple(h.parameters(bpy.context)['center']), (0, 0, 0))
        self.scene.cursor.location = (.001, .002, 0)
        bpy.ops.jointcraft.hinge_cursor()
        self.assertIsNone(self.scene.jc_hinge_center_object)
        self.assertAlmostEqual(h.parameters(bpy.context)['center'].x, .001)

    def test_failure_rolls_back_all_allocations(self):
        a, b = self.parts()
        original = (a.data, b.data)
        objects, meshes = set(bpy.data.objects), set(bpy.data.meshes)
        real = h.Stage.boolean
        calls = [0]

        def fail(stage, target, cutter, operation):
            calls[0] += 1
            if calls[0] == 9:
                raise RuntimeError('injected Boolean failure')
            return real(stage, target, cutter, operation)

        with patch.object(h.Stage, 'boolean', fail), self.assertRaisesRegex(RuntimeError, 'injected'):
            h.build(bpy.context, h.parameters(bpy.context))
        self.assertEqual((a.data, b.data), original)
        self.assertEqual(set(bpy.data.objects), objects)
        self.assertEqual(set(bpy.data.meshes), meshes)
        self.assertFalse(a.modifiers or b.modifiers)

    def test_unattached_center_refused_without_damage(self):
        a, b = self.parts()
        self.scene.jc_hinge_center = (0, .1, 0)
        original = (a.data, b.data)
        with self.assertRaises((ValueError, RuntimeError)):
            h.build(bpy.context, h.parameters(bpy.context))
        self.assertEqual((a.data, b.data), original)
        self.assertEqual(len(bpy.data.objects), 2)

    def test_final_pin_and_assignment_failures_roll_back(self):
        for failure in ('pin', 'assignment'):
            with self.subTest(failure=failure):
                self.setUp()
                a, b = self.parts()
                originals = (a.data, b.data)
                objects, meshes = set(bpy.data.objects), set(bpy.data.meshes)

                def assign(obj, mesh):
                    if obj == b:
                        raise RuntimeError('injected assignment failure')
                    obj.data = mesh

                target = 'create_pin_object' if failure == 'pin' else 'assign_mesh'
                effect = RuntimeError('injected pin failure') if failure == 'pin' else assign
                with patch.object(h, target, side_effect=effect), self.assertRaisesRegex(RuntimeError, 'injected'):
                    h.build(bpy.context, h.parameters(bpy.context))
                self.assertEqual((a.data, b.data), originals)
                self.assertEqual(set(bpy.data.objects), objects)
                self.assertEqual(set(bpy.data.meshes), meshes)

    def test_modifiers_shape_keys_and_linked_mesh_refused(self):
        a, b = self.parts()
        mod = a.modifiers.new('keep', 'BEVEL')
        with self.assertRaisesRegex(ValueError, 'modifiers'):
            h.parameters(bpy.context)
        a.modifiers.remove(mod)
        a.shape_key_add(name='Basis')
        with self.assertRaisesRegex(ValueError, 'shape keys'):
            h.parameters(bpy.context)
        a.shape_key_clear()
        folder = ROOT / 'artifacts'
        folder.mkdir(exist_ok=True)
        library = str(folder / 'hinge-test-library.blend')
        bpy.data.libraries.write(library, {a.data})
        with bpy.data.libraries.load(library, link=True) as (source, destination):
            destination.meshes = [source.meshes[0]]
        a.data = destination.meshes[0]
        with self.assertRaisesRegex(ValueError, 'Linked'):
            h.parameters(bpy.context)

    def test_flat_parts_nonuniform_negative_and_applied_transforms(self):
        for apply in (False, True):
            with self.subTest(applied=apply):
                self.setUp()
                a, b = self.parts(translation=(.04, -.02, .01))
                for obj in (a, b):
                    obj.scale = (-1, 1, .2)
                    obj.rotation_euler.z = math.pi
                if apply:
                    bpy.ops.object.select_all(action='SELECT')
                    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
                h.build(bpy.context, h.parameters(bpy.context))
                self.assertEqual(len(bpy.data.objects), 3)

    def test_fit_sizes_and_preserves_clearances_in_different_units(self):
        for scale in (1, .001):
            with self.subTest(scale=scale):
                self.setUp()
                a, b = self.parts(scale=scale)
                meshes = (a.data, b.data)
                settings = (self.scene.jc_hinge_radial_mm, self.scene.jc_hinge_axial_mm,
                            self.scene.jc_hinge_wall_mm)
                self.assertEqual(bpy.ops.jointcraft.hinge_fit(), {'FINISHED'})
                self.assertAlmostEqual(self.scene.jc_hinge_height_mm, 6.4, places=4)
                self.assertAlmostEqual(self.scene.jc_hinge_pin_mm, 2.4, places=4)
                self.assertEqual(settings, (self.scene.jc_hinge_radial_mm,
                                           self.scene.jc_hinge_axial_mm, self.scene.jc_hinge_wall_mm))
                self.assertEqual((a.data, b.data), meshes)
                self.assertEqual(len(bpy.data.objects), 2)
                h.build(bpy.context, h.parameters(bpy.context))

    def test_fit_large_standard_cubes(self):
        a, b = self.parts()
        for obj in (a, b):
            obj.location *= 100
            obj.scale = (100, 100, 100)
        self.assertEqual(bpy.ops.jointcraft.hinge_fit(), {'FINISHED'})
        self.assertAlmostEqual(self.scene.jc_hinge_height_mm, 1600, places=3)
        self.assertAlmostEqual(self.scene.jc_hinge_pin_mm, 600, places=3)
        h.build(bpy.context, h.parameters(bpy.context))
        for obj in bpy.data.objects:
            h.mesh_info(obj.data, obj.matrix_world)
        pin = next(obj for obj in bpy.data.objects if obj.get('jointcraft_role') == 'hinge_pin')
        original = b.matrix_world.copy()
        stage = h.Stage(bpy.context)
        try:
            for angle in (-30, -15, 15, 30):
                b.matrix_world = Matrix.Rotation(math.radians(angle), 4, 'Z') @ original
                self.assertLess(stage.overlap(b, pin), 1e-13)
        finally:
            b.matrix_world = original
            stage.close()

    def test_fit_failure_preserves_fields_and_explains_axial_mismatch(self):
        a, b = self.parts()
        for obj in (a, b):
            obj.scale.z = .2
        self.scene.jc_hinge_height_mm = 12
        with self.assertRaisesRegex(ValueError, 'outside the part thickness'):
            h.parameters(bpy.context)
        old = tuple(getattr(self.scene, name) for name in h.FIT_PROPERTIES)
        with self.assertRaisesRegex(RuntimeError, 'too thin'):
            bpy.ops.jointcraft.hinge_fit()
        self.assertEqual(tuple(getattr(self.scene, name) for name in h.FIT_PROPERTIES), old)
        self.assertEqual(len(bpy.data.objects), 2)


jc.register()
result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(HingeTests))
jc.unregister()
assert not hasattr(bpy.types.Scene, 'jc_hinge_a')
jc.register()
jc.unregister()
if not result.wasSuccessful():
    raise RuntimeError('Hinge tests failed')
