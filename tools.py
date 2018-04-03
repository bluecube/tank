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

def name_only_part(name, attributes=[]):
    return sphere() \
        .make_part(name, attributes) \
        .hidden()

if __name__ == "__main__":
    import codecad

    codecad.commandline_render(cone_with_rib(10, 5, 10, 20, 3, 5), 0.1)
