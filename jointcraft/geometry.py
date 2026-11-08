"""Axis-aligned bounds calculations, independent of Blender context."""
import math


def interface_from_bounds(pmin, pmax, smin, smax, tolerance):
    """Return the closest shared AABB face with positive overlapping area."""
    if not math.isfinite(tolerance) or tolerance < 0:
        raise ValueError("Face tolerance must be finite and non-negative")
    for low, high in ((pmin, pmax), (smin, smax)):
        if any(not math.isfinite(v) for v in (*low, *high)):
            raise ValueError("Mesh bounds must be finite")
        if any(hi <= lo for lo, hi in zip(low, high)):
            raise ValueError("Mesh must have non-zero thickness on every axis")
    # A small float32 precision floor, independent of print-fit clearance.
    magnitude = max(abs(v) for bounds in (pmin, pmax, smin, smax) for v in bounds)
    tolerance = max(tolerance, magnitude * 2**-22, 1e-9)
    choices = []
    for axis in range(3):
        others = [i for i in range(3) if i != axis]
        overlaps = [(max(pmin[i], smin[i]), min(pmax[i], smax[i])) for i in others]
        if any(hi - lo <= tolerance for lo, hi in overlaps):
            continue
        for sign, face_pin, face_socket in ((1, pmax[axis], smin[axis]),
                                            (-1, pmin[axis], smax[axis])):
            gap = sign * (face_socket - face_pin)
            if abs(gap) <= tolerance:
                area = math.prod(hi - lo for lo, hi in overlaps)
                choices.append((abs(gap), -area, axis, sign, face_pin, overlaps,
                                pmin, pmax, smin, smax))
    if not choices:
        raise ValueError(
            "No touching axis-aligned faces found (effective tolerance %.6g BU)" % tolerance)
    return min(choices, key=lambda v: (v[0], v[1]))
