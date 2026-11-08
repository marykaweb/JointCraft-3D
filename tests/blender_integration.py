"""blender -b --factory-startup --python-exit-code 1 --python tests/blender_integration.py"""
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import bpy
import bmesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import jointcraft as jc
sys.path.insert(0, str(ROOT / 'tests'))
from test_geometry import InterfaceTests


def volume(obj):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.transform(obj.matrix_world)
    try:
        assert all(edge.is_manifold for edge in bm.edges), obj.name
        return abs(bm.calc_volume(signed=True))
    finally:
        bm.free()


class BlenderTests(unittest.TestCase):
    def setUp(self):
        bpy.ops.wm.read_factory_settings(use_empty=True)
        self.scene = bpy.context.scene
        self.scene.unit_settings.scale_length = 1

    def cubes(self, axis=0, sign=1, size=2, location=(0, 0, 0)):
        bpy.ops.mesh.primitive_cube_add(size=size, location=location)
        pin = bpy.context.object
        bpy.ops.mesh.primitive_cube_add(size=size, location=location)
        socket = bpy.context.object
        bpy.context.view_layer.update()
        # Deliberately leave matrix_world stale, as in scripted scene setup.
        socket.location[axis] += sign * size
        self.scene.jc_pin, self.scene.jc_socket = pin, socket
        return pin, socket

    def create(self, pin, socket, radius, length):
        bpy.context.view_layer.update()
        before = [volume(obj) for obj in (pin, socket)]
        matrices = [obj.matrix_world.copy() for obj in (pin, socket)]
        self.assertEqual(bpy.ops.jointcraft.create_auto(), {'FINISHED'})
        added = 24 * math.sin(2 * math.pi / 48) * radius**2 * length
        clearance = jc.mm_to_bu(bpy.context, .2)
        removed = 24 * math.sin(2 * math.pi / 48) * (radius + clearance)**2 * (length + clearance)
        self.assertAlmostEqual(volume(pin) - before[0], added, delta=max(added * .003, 1e-13))
        self.assertAlmostEqual(before[1] - volume(socket), removed, delta=max(removed * .003, 1e-13))
        for obj, matrix in zip((pin, socket), matrices):
            self.assertEqual(obj.matrix_world, matrix)
            self.assertFalse(obj.modifiers)
        self.assertFalse(any(obj.name.startswith('JC_') for obj in bpy.data.objects))
        self.assertFalse(any(mesh.name.startswith('JC_') and mesh.users == 0 for mesh in bpy.data.meshes))

    def test_acceptance_cubes_all_six_directions(self):
        for axis in range(3):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    self.setUp()
                    pin, socket = self.cubes(axis, sign)
                    self.assertEqual(jc.find_interface(bpy.context, pin, socket, .00005)[2:4], (axis, sign))
                    self.create(pin, socket, .004, .006)

    def test_stale_location_is_updated_by_operator(self):
        pin, socket = self.cubes(size=.02)
        self.assertEqual(bpy.ops.jointcraft.create_auto(), {'FINISHED'})
        self.assertGreater(max(v.co.x for v in pin.data.vertices), .0159)

    def test_transforms_and_unequal_sizes(self):
        for apply in (False, True):
            with self.subTest(applied=apply):
                self.setUp()
                pin, socket = self.cubes(size=.02, location=(.1, .2, .3))
                pin.scale = (-1, 1.5, 1)
                socket.scale = (1, .75, .8)
                pin.rotation_euler.z = math.pi
                bpy.context.view_layer.update()
                if apply:
                    bpy.ops.object.select_all(action='SELECT')
                    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
                self.create(pin, socket, .003, .006)

    def test_units(self):
        for scale in (1, .001, .01):
            self.setUp()
            self.scene.unit_settings.scale_length = scale
            self.assertAlmostEqual(jc.mm_to_bu(bpy.context, .2), .0002 / scale)
            pin, socket = self.cubes(size=.02 / scale)
            self.create(pin, socket, .004 / scale, .006 / scale)

    def test_invalid_scene_object_and_empty_mesh(self):
        pin, socket = self.cubes()
        self.scene.collection.objects.unlink(socket)
        self.assertFalse(jc.mesh_in_scene(self.scene, socket))
        with self.assertRaisesRegex(ValueError, 'not in the active scene'):
            jc.find_interface(bpy.context, pin, socket, .00005)
        self.scene.collection.objects.link(socket)
        self.assertTrue(jc.mesh_in_scene(self.scene, socket))
        socket.data = bpy.data.meshes.new('empty')
        with self.assertRaisesRegex(ValueError, 'no mesh vertices'):
            jc.find_interface(bpy.context, pin, socket, .00005)

    def test_no_contact_diagnostic_and_no_mutation(self):
        pin, socket = self.cubes()
        socket.location.x = 3
        original = (pin.data, socket.data)
        with self.assertRaisesRegex(ValueError, "Pin 'Cube'.*Socket 'Cube.001'.*mm"):
            jc.find_interface(bpy.context, pin, socket, .00005)
        with self.assertRaisesRegex(RuntimeError, 'No touching'):
            bpy.ops.jointcraft.create_auto()
        self.assertEqual((pin.data, socket.data), original)

    def test_second_boolean_failure_rolls_back(self):
        pin, socket = self.cubes()
        original = (pin.data, socket.data)
        objects, meshes = set(bpy.data.objects), set(bpy.data.meshes)
        real_boolean = jc.boolean_mesh

        def fail_socket(context, target, cutter, operation):
            if operation == 'DIFFERENCE':
                raise RuntimeError('injected socket failure')
            return real_boolean(context, target, cutter, operation)

        with patch.object(jc, 'boolean_mesh', side_effect=fail_socket):
            with self.assertRaisesRegex(RuntimeError, 'injected socket failure'):
                bpy.ops.jointcraft.create_auto()
        self.assertEqual((pin.data, socket.data), original)
        self.assertEqual(set(bpy.data.objects), objects)
        self.assertEqual(set(bpy.data.meshes), meshes)
        self.assertFalse(pin.modifiers or socket.modifiers)

    def test_cylinder_normals(self):
        for axis in range(3):
            for sign in (-1, 1):
                mesh = jc.cylinder_mesh('normals', (0, 0, 0), axis, sign, -.003, .006, .004)
                bm = bmesh.new()
                bm.from_mesh(mesh)
                self.assertGreater(bm.calc_volume(signed=True), 0)
                self.assertTrue(all(edge.is_manifold for edge in bm.edges))
                bm.free()
                bpy.data.meshes.remove(mesh)


jc.register()
print('Testing Blender', bpy.app.version_string, flush=True)
suite = unittest.TestSuite([
    unittest.defaultTestLoader.loadTestsFromTestCase(InterfaceTests),
    unittest.defaultTestLoader.loadTestsFromTestCase(BlenderTests),
])
result = unittest.TextTestRunner(verbosity=2).run(suite)
jc.unregister()
if not result.wasSuccessful():
    raise RuntimeError('JointCraft regression tests failed')
