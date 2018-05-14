from codecad.shapes import *

def cone(height, upper_diameter, lower_diameter, base_height=0):
    """ Truncated cone with cylindrical base, standing on XY plane """
    points = [(-lower_diameter / 2, 0),
              (lower_diameter / 2, 0)]

    if base_height > 0:
        points.append((lower_diameter / 2, base_height))

    points += [(upper_diameter / 2, height + base_height),
               (-lower_diameter / 2, height + base_height)]

    return polygon2d(points).revolved().rotated_x(90)

def rib(height, upper_length, lower_length, width, base_height=0):
    rib_points = [(0, 0),
                  (lower_length, 0)]
    if base_height > 0:
        rib_points.append((lower_length, base_height))

    rib_points += [(upper_length, height + base_height),
                   (0, height + base_height)]

    return polygon2d(rib_points).extruded(width).rotated_x(90)

def cone_with_rib(height, upper_diameter, lower_diameter, rib_length, rib_width, base_height=0):
    """ Truncated cone with cylindrical base, extended by a rib going in X direction. """
    return cone(height, upper_diameter, lower_diameter, base_height) + \
        rib(height, upper_diameter / 2, rib_length, rib_width, base_height)
        #rib(height, math.sqrt(upper_diameter**2 - rib_width**2) / 2, rib_length, rib_width, base_height)

def spline(outer_diameter, approx_tooth_size = 2):
    n = int(math.pi * (outer_diameter - approx_tooth_size / 2) / (2 * approx_tooth_size))
    tooth_radius = outer_diameter / (2 * math.cos(math.pi / (2 * n)))
    tooth_size = 2 * tooth_radius * math.sin(math.pi / (2 * n))

    spline = circle(d=outer_diameter)
    spline -= circle(d=tooth_size).translated_x(tooth_radius)
    spline = unsafe.CircularRepetition2D(spline, n)

    spline.n = n
    spline.od = outer_diameter
    spline.id = 2 * (tooth_radius - tooth_size / 2)
    spline.tooth_size = tooth_size

    return spline

def crown(outer_diameter, inner_diameter=None,
          inverse=False,
          assymetrical=True, approx_tooth_size=5, min_n=2):
    if inner_diameter is None:
        inner_diameter = 0.6 * outer_diameter
    else:
        assert inner_diameter < outer_diameter

    n = max(min_n,
            round(math.pi * inner_diameter / (2 * approx_tooth_size)))
    mask = circle(d=outer_diameter) - circle(d=inner_diameter)

    def segment(angle):
        return half_plane().rotated(180 + angle / 2) & half_plane().rotated(-angle / 2)

    ret = unsafe.CircularRepetition2D(segment(180 / n), n)
    if assymetrical:
        ret += segment(1.5 * 180 / n)

    if inverse:
        ret = mask - ret
    else:
        ret = mask & ret

    ret.n = n
    ret.od = outer_diameter
    ret.id = inner_diameter

    return ret

def crown_cutout(outer_diameter, inner_diameter, tolerance, height, inverse):
    c = crown(outer_diameter=1.2 * outer_diameter,
              inner_diameter=inner_diameter - tolerance,
              inverse=inverse)
    ret = c \
        .offset(tolerance / 2) \
        .extruded(2 * height)
    ret += cylinder(d=inner_diameter, h=height)

    return ret

def wheel_lightening_holes(n, inner_radius, outer_radius, wall_thickness):
    center_radius = (inner_radius + outer_radius) / 2
    polygon = regular_polygon2d(n=n, r=center_radius)
    hole_diameter = min(outer_radius - inner_radius,
                        polygon.side_length - wall_thickness)

    return unsafe.CircularRepetition2D(circle(d=hole_diameter).translated_x(center_radius),
                                       n).extruded(float("inf"))

def name_only_part(name, attributes=[]):
    return sphere() \
        .make_part(name, attributes) \
        .hidden()

if __name__ == "__main__":
    import codecad
    s = crown(30)

    codecad.commandline_render(s)
