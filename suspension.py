import math

import codecad
from codecad.shapes import *

import parameters

def x(y):
    print(y)
    return y

def road_wheel_half_generator(diameter, width, bearing_inset,
                              guide_width, guide_height, guide_angle, guide_clearance,
                              bearing_od, bearing_thickness, bearing_shoulder_size,
                              o_ring_minor_diameter, wall_thickness):

    o_ring_protrusion = o_ring_minor_diameter / 2
    radius = diameter / 2 - o_ring_protrusion
    bearing_radius = bearing_od / 2


    cos_guide_angle = math.cos(math.radians(guide_angle))
    guide_groove_base_half_width = guide_width / 2 + guide_clearance - cos_guide_angle * o_ring_protrusion
    guide_groove_top_radius = diameter / 2 - guide_height - guide_clearance
    guide_groove_top_half_width = guide_width / 2 + guide_clearance - (guide_height + guide_clearance) * cos_guide_angle

    half = polygon2d(([(bearing_radius - bearing_shoulder_size, 0),
                       (bearing_radius - bearing_shoulder_size,
                        width / 2 - bearing_thickness - bearing_inset),
                       (bearing_radius,
                        width / 2 - bearing_thickness - bearing_inset),
                       (bearing_radius, width / 2 - bearing_inset),
                       (bearing_radius + bearing_inset, width / 2),
                       (radius, width / 2),
                       (radius, guide_groove_base_half_width),
                       (guide_groove_top_radius, guide_groove_top_half_width),
                       (guide_groove_top_radius, 0),
                       ]))

    o_ring_count_per_side = 2
    o_ring_side_size = width / 2 - guide_groove_base_half_width
    o_ring_spacing = o_ring_side_size / (1 + o_ring_count_per_side)
    o_ring_groove_diameter = 1.1 * o_ring_minor_diameter
    for i in range(o_ring_count_per_side):
        pos = width / 2 - (i + 1) * o_ring_spacing
        half -= circle(d=o_ring_groove_diameter).translated(radius, pos)

    half = half.revolved().rotated_x(90)

    lightening_hole_diameter = guide_groove_top_radius - bearing_radius - 2 * wall_thickness
    lightening_hole_center_radius = (guide_groove_top_radius + bearing_radius) / 2
    lightening_hole_count = math.floor(math.pi / math.asin((lightening_hole_diameter + wall_thickness) / lightening_hole_center_radius / 2))

    half -= cylinder(d=lightening_hole_diameter, h=float("inf")) \
            .translated_x(lightening_hole_center_radius) \
            .rotated((0, 0, 1), 360, n=lightening_hole_count)

    return half

def suspension_arm_generator(dx, dy, thickness, height):
    pass

road_wheel_inner_half = road_wheel_half_generator(parameters.road_wheel_diameter,
                                                  parameters.road_wheel_width,
                                                  parameters.road_wheel_inner_inset,
                                                  parameters.tread_guide_width,
                                                  parameters.tread_guide_height,
                                                  parameters.tread_guide_side_angle,
                                                  parameters.tread_guide_clearance,
                                                  parameters.small_bearing_od,
                                                  parameters.small_bearing_thickness,
                                                  parameters.small_bearing_shoulder_size,
                                                  parameters.road_wheel_o_ring_minor_diameter,
                                                  parameters.thin_wall).make_part("road_wheel_inner_half")
road_wheel_outer_half = road_wheel_half_generator(parameters.road_wheel_diameter,
                                                  parameters.road_wheel_width,
                                                  parameters.road_wheel_outer_inset,
                                                  parameters.tread_guide_width,
                                                  parameters.tread_guide_height,
                                                  parameters.tread_guide_side_angle,
                                                  parameters.tread_guide_clearance,
                                                  parameters.small_bearing_od,
                                                  parameters.small_bearing_thickness,
                                                  parameters.small_bearing_shoulder_size,
                                                  parameters.road_wheel_o_ring_minor_diameter,
                                                  parameters.thin_wall).make_part("road_wheel_outer_half")

road_wheel = codecad.Assembly([road_wheel_inner_half.rotated_x(180),
                               road_wheel_outer_half
                               ]).make_part("road_wheel")

suspension = codecad.Assembly()
suspension.add(road_wheel)
suspension.add(road_wheel.translated_x(50))
suspension.add(road_wheel.translated_x(100))

if __name__ == "__main__":
    codecad.commandline_render(suspension, 0.1)
