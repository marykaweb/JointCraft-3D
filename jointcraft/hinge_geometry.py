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
    # Circumscribed cut polygons keep the nominal circular clearance at ANY
    # rotation, including large pins whose facet sagitta exceeds the print gap.
    cosine = math.cos(math.pi / 64)
    hole_cut = hole / cosine
    outer = outer_diameter / 2 if outer_diameter else (hole_cut + wall) / cosine
    middle = middle_width or (height - 2 * axial) / 3
    ear = (height - middle - 2 * axial) / 2
    if outer * cosine < hole_cut + wall - 1e-12:
        raise ValueError('Outer diameter leaves less than the requested wall')
    if min(middle, ear) < wall:
        raise ValueError('Increase hinge height: each knuckle must be at least Minimum wall thick')
    shaft_length = pin_length or height + 2 * axial + wall
    if shaft_length < height + 2 * axial + wall * .5:
        raise ValueError('Pin shaft is too short for the hinge and exposed glue tip')
    attachment_wall = outer - hole_cut
    head_thickness = max(wall, pin_diameter * .15)
    return dict(pin_radius=pin_diameter / 2, hole=hole, hole_cut_radius=hole_cut,
                relief_radius=(outer + axial) / cosine, outer=outer,
                axial=axial, height=height, wall=wall, middle=middle, ear=ear,
                shaft_length=shaft_length, angle=angle,
                intervals=((-height / 2, -middle / 2 - axial),
                           (-middle / 2, middle / 2),
                           (middle / 2 + axial, height / 2)),
                reach=outer + axial + 2 * attachment_wall, support_half=outer * .55,
                head_radius=hole + attachment_wall * .65, head_thickness=head_thickness)


def fit_dimensions(available_height, side_depth, side_half_width, wall, radial, axial,
                   percent=80):
    """Proportional starting dimensions, bounded by the local box envelope."""
    if not 10 <= percent <= 100 or min(available_height, side_depth, side_half_width,
                                       wall, radial, axial) <= 0:
        raise ValueError('Center must lie inside the shared thickness and width of both parts')
    minimum_height = (3 * wall + 2 * axial) * 1.001
    if minimum_height > available_height:
        raise ValueError('Parts are too thin for three knuckles with these wall/axial clearances')
    height = max(available_height * percent / 100, minimum_height)
    transverse_limit = side_half_width * .7
    cosine = math.cos(math.pi / 64)
    radial_factor = 3 / cosine**2 - 2 / cosine
    diameter = min(height * .375,
                   (side_depth - axial - radial * radial_factor - 3 * wall / cosine) / (.5 * radial_factor),
                   (side_depth - axial - radial * radial_factor) / (.5 * radial_factor + .9 / cosine),
                   (transverse_limit - radial / cosine**2 - wall / cosine) / (.5 / cosine**2),
                   (transverse_limit - radial / cosine**2) / (.5 / cosine**2 + .3 / cosine))
    if diameter <= 0:
        raise ValueError('Not enough room around center for the current wall and clearances')
    outer = ((diameter / 2 + radial) / cosine + max(wall, diameter * .3)) / cosine
    return dict(height=height, pin_diameter=diameter, outer_diameter=2 * outer)


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
    return lathe(((bottom - p['head_thickness'], p['head_radius']),
                  (bottom, p['head_radius']), (bottom, p['pin_radius']),
                  (bottom + p['shaft_length'], p['pin_radius'])))


def world_surface(surface, center, u, v, w):
    vertices, faces = surface
    return ([tuple(center[i] + x * u[i] + y * v[i] + z * w[i] for i in range(3))
             for x, y, z in vertices], faces)
