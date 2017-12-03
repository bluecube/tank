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


def suspension_arm_generator(right,
                             dx, dy, thickness, height,
                             axle_diameter,
                             spring_angle, knob_height,
                             bearing_id, bearing_shoulder_size):
    # Position 0, 0 is the wheel center

    if right:
        dx = -dx

    bend_distance = 12
    bend_x = math.copysign(math.cos(math.radians(spring_angle)) * bend_distance, dx)
    bend_y = math.sin(math.radians(spring_angle)) * bend_distance

    arm = union([capsule(0, 0, -bend_x, bend_y, height),
                 capsule(-bend_x, bend_y, -dx, dy, height)]) # TODO: rounding
    arm = arm.extruded(thickness, symmetrical=False)

    arm += polygon2d([(0, 0), (0, knob_height),
                      (bearing_id / 2 + bearing_shoulder_size, knob_height),
                      (height / 2, 0)]) \
        .revolved() \
        .rotated_x(90) \
        .translated_z(thickness)

    hole = cylinder(d=axle_diameter, h=float("inf"))

    arm -= hole
    arm -= hole.translated(-dx, dy, 0)

    return arm


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
                                                  parameters.thin_wall
                                                  ).make_part("road_wheel_inner_half", ["3d_print"])
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
                                                  parameters.thin_wall
                                                  ).make_part("road_wheel_outer_half", ["3d_print"])
road_wheel = codecad.Assembly([road_wheel_inner_half.rotated_x(180),
                               road_wheel_outer_half
                               ]).make_part("road_wheel")

def make_wheel_suspension(right):
    name = "right" if right else "left"

    assert parameters.spring_lower_mount_diameter == parameter.small_screw_diameter

    suspension_arm = suspension_arm_generator(right,
                                              parameters.suspension_arm_dx,
                                              parameters.suspension_arm_dy,
                                              parameters.suspension_arm_thickness,
                                              parameters.suspension_arm_height,
                                              parameters.small_screw_diameter,
                                              parameters.suspension_spring_angle,
                                              parameters.suspension_arm_wheel_clearance + parameters.road_wheel_inner_inset,
                                              parameters.small_bearing_id,
                                              parameters.small_bearing_shoulder_size
                                              ).make_part("{}_suspension_arm".format(name), ["3d_print"])

    return codecad.Assembly([road_wheel.rotated_x(90),
                             suspension_arm.rotated_x(90) \
                             .translated_y(parameters.road_wheel_width / 2 +
                                            parameters.road_wheel_arm_clearance +
                                            parameters.suspension_arm_thickness)
                             ]).make_part("{}_suspension".format(name))

left_wheel_suspension = make_wheel_suspension(False)
right_wheel_suspension = make_wheel_suspension(True)


if __name__ == "__main__":
    codecad.commandline_render(left_wheel_suspension.shape().rotated_z(-90) & half_space(), 0.1)
