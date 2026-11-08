"""Run without bpy: python -m unittest discover -s tests -p test_geometry.py."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'geometry', Path(__file__).resolve().parents[1] / 'jointcraft' / 'geometry.py')
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)
find = geometry.interface_from_bounds


class InterfaceTests(unittest.TestCase):
    def test_all_directions(self):
        for axis in range(3):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    low, high = [-1.] * 3, [1.] * 3
                    low[axis] += sign * 2
                    high[axis] += sign * 2
                    result = find([-1.] * 3, [1.] * 3, low, high, 0.00005)
                    self.assertEqual(result[2:5], (axis, sign, float(sign)))

    def test_unequal_parts_and_translation(self):
        result = find([9, 19, 29], [11, 21, 31], [11, 20, 30], [12, 23, 34], 1e-5)
        self.assertEqual(result[5], [(20, 21), (30, 31)])

    def test_tolerance(self):
        for gap in (-0.00004, 0, 0.00004):
            find([-1] * 3, [1] * 3, [1 + gap, -1, -1], [3 + gap, 1, 1], 0.00005)
        for gap in (-0.001, 0.001):
            with self.assertRaisesRegex(ValueError, 'No touching'):
                find([-1] * 3, [1] * 3, [1 + gap, -1, -1], [3 + gap, 1, 1], 0.00005)

    def test_edge_corner_and_intersection_are_not_faces(self):
        for low, high in (([1, 1, -1], [3, 3, 1]), ([1, 1, 1], [3, 3, 3]),
                          ([0, -1, -1], [2, 1, 1])):
            with self.assertRaises(ValueError):
                find([-1] * 3, [1] * 3, low, high, 0.00005)

    def test_small_parts(self):
        result = find([-.01] * 3, [.01] * 3, [.01, -.01, -.01], [.03, .01, .01], .00005)
        self.assertEqual(result[2:4], (0, 1))

    def test_invalid_values(self):
        for tol in (-1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                find([-1] * 3, [1] * 3, [1, -1, -1], [3, 1, 1], tol)
        with self.assertRaises(ValueError):
            find([0] * 3, [0] * 3, [1] * 3, [2] * 3, .001)


if __name__ == '__main__':
    unittest.main()
