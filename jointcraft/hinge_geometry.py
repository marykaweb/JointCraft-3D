"""Hinge dimensions and closed primitive surfaces; no bpy or scene state."""
import math


def dimensions(pin_diameter, radial, axial, height, wall, outer_diameter=0,
               middle_width=0, pin_length=0, angle=30):
    values = (pin_diameter, radial, axial, height, wall, outer_diameter,
              middle_width, pin_length, angle)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Dimensions must be finite')
    if min(pin_diameter, radial, axial, height, wall) <= 0:
        raise ValueError('Diameter, height, wall and clearances must be positive')
    if min(outer_diameter, middle_width, pin_length) < 0 or not 5 <= angle <= 60:
        raise ValueError('Invalid advanced dimensions or rotation range (5 to 60 degrees)')
    hole = pin_diameter / 2 + radial
    outer = outer_diameter / 2 if outer_diameter else hole + wall
    middle = middle_width or (height - 2 * axial) / 3
    ear = (height - middle - 2 * axial) / 2
    if outer < hole + wall - 1e-12:
        raise ValueError('Outer diameter leaves less than the requested wall')
    if min(middle, ear) < wall:
        raise ValueError('Increase hinge height: each knuckle must be at least Minimum wall thick')
    shaft_length = pin_length or height + 2 * axial + wall
    if shaft_length < height + 2 * axial + wall * .5:
        raise ValueError('Pin shaft is too short for the hinge and exposed glue tip')
    return dict(pin_radius=pin_diameter / 2, hole=hole, outer=outer,
                axial=axial, height=height, wall=wall, middle=middle, ear=ear,
                shaft_length=shaft_length, angle=angle,
                intervals=((-height / 2, -middle / 2 - axial),
                           (-middle / 2, middle / 2),
                           (middle / 2 + axial, height / 2)),
                reach=outer + axial + 2 * wall, support_half=outer * .55,
                head_radius=hole + wall * .65)


def lathe(profile, segments=64):
    vertices = [(radius * math.cos(math.tau * j / segments),
                 radius * math.sin(math.tau * j / segments), z)
                for z, radius in profile for j in range(segments)]
    faces = [tuple(reversed(range(segments)))]
    for ring in range(len(profile) - 1):
        for j in range(segments):
            k = (j + 1) % segments
            a, b = ring * segments, (ring + 1) * segments
            faces.append((a + j, a + k, b + k, b + j))
    faces.append(tuple(range((len(profile) - 1) * segments, len(profile) * segments)))
    return vertices, faces


def cylinder(radius, start, end):
    return lathe(((start, radius), (end, radius)))


def prism(polygon, low, high):
    n = len(polygon)
    vertices = [(x, y, z) for z in (low, high) for x, y in polygon]
    faces = [tuple(reversed(range(n))), tuple(range(n, 2 * n))]
    faces += [(j, (j + 1) % n, (j + 1) % n + n, j + n) for j in range(n)]
    return vertices, faces


def support(p, side, start, end):
    lo, hi = sorted((side * p['outer'] * .4, side * p['reach']))
    w = p['support_half']
    return prism(((lo, -w), (hi, -w), (hi, w), (lo, w)), start, end)


def keeper(p, side, extent):
    """Keep the original body in a wedge providing symmetric swing room."""
    margin = p['axial'] / 2
    bevel = math.tan(math.radians(p['angle'] / 2)) * extent
    poly = [(-extent, -extent), (-margin - bevel, -extent),
            (-margin, 0), (-margin - bevel, extent), (-extent, extent)]
    if side > 0:
        poly = [(-x, y) for x, y in reversed(poly)]
    return prism(poly, -extent, extent)


def pin(p):
    bottom = -p['height'] / 2 - p['axial']
    return lathe(((bottom - p['wall'], p['head_radius']),
                  (bottom, p['head_radius']), (bottom, p['pin_radius']),
                  (bottom + p['shaft_length'], p['pin_radius'])))


def world_surface(surface, center, u, v, w):
    vertices, faces = surface
    return ([tuple(center[i] + x * u[i] + y * v[i] + z * w[i] for i in range(3))
             for x, y, z in vertices], faces)
