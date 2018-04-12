import math

import codecad
from codecad.shapes import *

import parameters
import suspension
import tools

suspension_pivot_z = 20
thin_wall = 5 * parameters.extrusion_width

# Depth of the shoulder screw shaft in the side panel, including the spacing knob
pivot_screw_diameter2_depth = parameters.shoulder_screw_length - parameters.shoulder_screw_screw_length \
    - suspension.pivot_guide_length - suspension.pivot_flat_clearance

side_thickness = pivot_screw_diameter2_depth - suspension.arm_clearance + thin_wall + \
    max(parameters.large_screw_nut_height, parameters.shoulder_screw_nut_height) - suspension.pivot_flat_clearance
side_pivot_height = suspension.arm_clearance - suspension.spring_anchor_point.z

def side_generator(width, height, thickness,
                   arm_clearance,
                   suspension_pivot_z, suspension_pivot_diameter1, suspension_pivot_diameter2,
                   suspension_pivot_diameter2_depth,
                   suspension_pivot_height, suspension_pivot_nut_s,

                   spring_anchor_point, spring_up_point, spring_down_point, spring_anchor_diameter,
                   spring_anchor_nut_s,

                   thin_wall,
                   hole_blinding_layer_height):

    side = box(width, height, thickness) \
        .translated(spring_anchor_point.x / 2, height / 2 - suspension_pivot_z, thickness / 2)

    side += tools.cone(height=suspension_pivot_height,
                       upper_diameter=suspension_pivot_diameter2 + 2 * thin_wall,
                       lower_diameter=suspension_pivot_diameter2 + 2 * thin_wall + 2 * suspension_pivot_height,
                       base_height=thickness)
    side += tools.cone(height=arm_clearance,
                       upper_diameter=spring_anchor_diameter + 2 * thin_wall,
                       lower_diameter=spring_anchor_diameter + 2 * thin_wall + 2 * arm_clearance,
                       base_height=thickness) \
        .translated(spring_anchor_point.x, spring_anchor_point.y, 0)

    spring_up_vector = spring_up_point - spring_anchor_point
    spring_up_angle = math.degrees(math.atan2(spring_up_vector.y, spring_up_vector.x))
    spring_down_vector = spring_down_point - spring_anchor_point
    spring_down_angle = math.degrees(math.atan2(spring_down_vector.y, spring_down_vector.x))
    side -= suspension.spring_cutout_generator(spring_angle=spring_up_angle - spring_down_angle,
                                               r0=4,
                                               r1=suspension.spring_length,
                                               chamfer0=0,
                                               chamfer1=suspension.spring_diameter / 2) \
        .rotated_z(spring_down_angle) \
        .translated(spring_anchor_point.x, spring_anchor_point.y, thickness + arm_clearance + suspension.spring_top_mount_thickness / 2)

    # Nut depth is measured from the back side
    nut_depth = thickness + arm_clearance - suspension_pivot_diameter2_depth - thin_wall

    side -= cylinder(d=suspension_pivot_diameter1, h=float("inf"))
    side -= cylinder(d=suspension_pivot_diameter2, h=2 * suspension_pivot_diameter2_depth) \
        .translated_z(thickness + arm_clearance)
    side -= regular_polygon2d(n=6, d=suspension_pivot_nut_s * 2 / math.sqrt(3)) \
        .extruded(2 * nut_depth)

    side -= cylinder(d=spring_anchor_diameter, h=float("inf")) \
        .translated(spring_anchor_point.x, spring_anchor_point.y, 0)
    side -= regular_polygon2d(n=6, d=spring_anchor_nut_s * 2 / math.sqrt(3)) \
        .extruded(2 * nut_depth) \
        .translated(spring_anchor_point.x, spring_anchor_point.y, 0)

    if hole_blinding_layer_height:
        side += cylinder(d=suspension_pivot_nut_s * 2, h=hole_blinding_layer_height, symmetrical=False) \
            .translated_z(nut_depth)
        side += cylinder(d=spring_anchor_nut_s * 2, h=hole_blinding_layer_height, symmetrical=False) \
            .translated(spring_anchor_point.x, spring_anchor_point.y, nut_depth)

    side = side.translated_y(suspension_pivot_z)

    return side

test_side = side_generator(80, 35, side_thickness,
                           suspension.arm_clearance,

                           suspension_pivot_z,
                           parameters.shoulder_screw_diameter, parameters.shoulder_screw_diameter2,
                           pivot_screw_diameter2_depth,
                           side_pivot_height, parameters.shoulder_screw_nut_s,

                           suspension.spring_anchor_point, suspension.spring_up_point, suspension.spring_down_point,
                           suspension.spring_top_mount_id, parameters.large_screw_nut_s,

                           thin_wall,
                           parameters.layer_height
                           ).make_part("test_side")

if __name__ == "__main__":
    codecad.commandline_render(test_side.rotated_x(90))
