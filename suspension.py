import sys

import warnings
import math

import codecad
from codecad.shapes import *

import util
import parameters
import track

bogie_count = 6 # Count of bogies on both sides of the vehicle

spring_length = 62 # Center to center, relaxed
spring_travel = 11
spring_diameter = 17.5
spring_top_mount_diameter = 5
spring_bottom_mount_diameter = 3
spring_top_mount_thickness = 3.8
spring_bottom_mount_thickness = 6.5
spring_preload_force = 0.95 # [kg]
spring_full_compression_force = 4.5 # [kg]

o_ring_minor_diameter = 2

arm_clearance = 1
clearance = 2

bogie_spacing = 110 # [mm] distance between bogies
bogie_wheel_spacing = 50 # [mm] distance between wheels of one bogie
wheel_diameter = 30
wheel_width = 30 # Total width of the wheel pair
arm_width = 8
bogie_width = arm_width

suspension_travel = 30 # [mm]
suspension_sag = 0.3 # Ratio of travel from neutral position down

half_wheel_width = (wheel_width - bogie_width) / 2 - arm_clearance

assert wheel_width - 2 * half_wheel_width >= track.guide_width + track.clearance, \
    "The track guide has enough space between the wheel halves"
assert wheel_width <= track.width

arm_thickness = parameters.shoulder_screw_diameter2 + 12 * parameters.extrusion_width

def road_wheel_generator(diameter, width, axle_diameter,
                         shoulder_height, shoulder_width,
                         o_ring_minor_diameter, wall_thickness, hole_blinding_layer_height,
                         screw_hole_diameter, screw_hole_depth,
                         hex_hole):

    o_ring_protrusion = o_ring_minor_diameter / 2
    radius = diameter / 2 - o_ring_protrusion
    axle_radius = axle_diameter / 2

    wheel = polygon2d([(axle_radius, 0),
                       (radius, 0),
                       (radius, width),
                       (axle_radius + shoulder_width + shoulder_height, width),
                       (axle_radius + shoulder_width, width + shoulder_height),
                       (axle_radius, width + shoulder_height),
                       ])

    o_ring_count = 2
    o_ring_spacing = (width - o_ring_count * o_ring_minor_diameter) / (1 + o_ring_count)
    for i in range(o_ring_count):
        pos = (i + 1) * o_ring_spacing + (i + 0.5) * o_ring_minor_diameter
        wheel -= circle(d=o_ring_minor_diameter).translated(radius, pos)

    wheel = wheel.revolved().rotated_x(90)

    if hole_blinding_layer_height:
        wheel += cylinder(r=radius - o_ring_protrusion,
                          h=hole_blinding_layer_height,
                          symmetrical=False).translated_z(screw_hole_depth)

    if hex_hole:
        wheel -= regular_polygon2d(n=6, d=screw_hole_diameter * 2 / math.sqrt(3)).extruded(2 * screw_hole_depth)
    else:
        wheel -= cylinder(d=screw_hole_diameter, h=2 * screw_hole_depth)

    lightening_hole_count = 5
    lightening_hole_inner_radius = max(axle_radius + shoulder_height + shoulder_width,
                                       axle_radius + wall_thickness,
                                       screw_hole_diameter / 2 + wall_thickness)
    lightening_hole_outer_radius = radius - o_ring_protrusion - wall_thickness
    lightening_hole_center_radius = (lightening_hole_inner_radius + lightening_hole_outer_radius) / 2
    lightening_hole_polygon = regular_polygon2d(n=lightening_hole_count,
                                                r=lightening_hole_center_radius)
    lightening_hole_diameter = min(lightening_hole_outer_radius - lightening_hole_inner_radius,
                                   lightening_hole_polygon.side_length - wall_thickness)

    wheel -= cylinder(d=lightening_hole_diameter, h=float("inf")) \
        .translated_x(lightening_hole_center_radius) \
        .rotated((0, 0, 1), 360, n=lightening_hole_count)

    return wheel

def bogie_generator(wheel_spacing, lower_thickness, upper_thickness,
                    bearing_diameter, bearing_thickness, bearing_shoulder_size,
                    thin_wall, thick_wall,
                    pivot_z,
                    wheel_cutout_diameter,
                    arm_cutout_diameter,
                    arm_cutout_thickness,
                    arm_cutout_angle,
                    shoulder_screw_diameter,
                    shoulder_screw_diameter2,
                    shoulder_screw_length,
                    shoulder_screw_screw_length,
                    shoulder_screw_head_diameter,
                    shoulder_screw_head_height,
                    shoulder_screw_nut_height,
                    shoulder_screw_nut_s,
                    overhang_angle):

    assert arm_cutout_angle < 180

    bogie = polygon2d([(-wheel_spacing / 2, 0),
                       (wheel_spacing / 2, 0),
                       (0, pivot_z)]) \
        .offset((bearing_diameter + thin_wall + thick_wall) / 2) \
        .extruded(upper_thickness) \
        .rotated_x(90) \
        .translated_z((thick_wall - thin_wall) / 2)

    bottom_z = -bearing_diameter / 2 - thin_wall

    nut_outer_diameter = shoulder_screw_nut_s * 2 / math.sqrt(3)
    pivot_end_diameter = max(shoulder_screw_diameter2, nut_outer_diameter) + 2 * thick_wall

    bogie += cylinder(d=pivot_end_diameter, h=upper_thickness) \
        .rotated_x(90) \
        .translated_z(pivot_z)

    cutout_tmp_point = (wheel_spacing * math.sin(math.radians(arm_cutout_angle / 2)), wheel_spacing * math.cos(math.radians(arm_cutout_angle / 2)))
    screw_sink_in = -2#(upper_thickness - (shoulder_screw_length + shoulder_screw_head_height)) / 2
    screw_head_plane_y = - (shoulder_screw_length - shoulder_screw_screw_length) / 2
    nut_plane_y = screw_head_plane_y + shoulder_screw_length - shoulder_screw_nut_height - shoulder_screw_diameter / 6

    # Screw head
    bogie -= cylinder(d=shoulder_screw_head_diameter, h=upper_thickness, symmetrical=False) \
        .rotated_x(90) \
        .translated(0, screw_head_plane_y, pivot_z)
    # Smooth part
    bogie -= cylinder(d=shoulder_screw_diameter2, h=2 * (shoulder_screw_length - shoulder_screw_screw_length)) \
        .rotated_x(90) \
        .translated(0, screw_head_plane_y, pivot_z)
    # Screw part
    bogie -= cylinder(d=shoulder_screw_diameter, h=float("inf")) \
        .rotated_x(90) \
        .translated_z(pivot_z)
    # Nut
    bogie -= regular_polygon2d(n=6, d=nut_outer_diameter) \
        .extruded(upper_thickness, symmetrical=False) \
        .rotated_x(-90) \
        .translated(0, nut_plane_y, pivot_z)

    # Space for the arm
    bogie -= polygon2d([(0, 0),
                        cutout_tmp_point,
                        (-cutout_tmp_point[0], cutout_tmp_point[1])]) \
        .offset(arm_cutout_diameter / 2) \
        .extruded(lower_thickness) \
        .rotated_x(90) \
        .translated_z(pivot_z)

    # Wheel cutouts
    for x in [-1, 1]:
        x = x * wheel_spacing / 2
        bogie -= cylinder(d=bearing_diameter - 2 * bearing_shoulder_size, h=lower_thickness).rotated_x(90).translated_x(x)
        for y in [-1, 1]:
            bogie -= cylinder(d=wheel_cutout_diameter, h=upper_thickness - lower_thickness + 1e-2).rotated_x(90).translated(x, y * upper_thickness / 2, 0)
            bogie -= cylinder(d=bearing_diameter, h=2 * bearing_thickness).rotated_x(90).translated(x, y * lower_thickness / 2, 0)

    # bottom lightening angles
    for y in [-1, 1]:
        bogie -= half_space().rotated_x(90 + y * (90 + overhang_angle)).translated(0, -y * lower_thickness / 2, bottom_z)

    return bogie

def spring_placeholder_generator(length): # Redo the geometry
    mounts = capsule(0, 0, 0, length, 8) - \
             circle(d=spring_bottom_mount_diameter) - \
             circle(d=spring_top_mount_diameter).translated_y(length)
    mounts = mounts.extruded(spring_top_mount_thickness).rotated_x(90)
    body = cylinder(d=spring_diameter, h=length * 0.7).translated_z(length * 0.55)
    return mounts + body


inner_road_wheel = road_wheel_generator(wheel_diameter,
                                        half_wheel_width,
                                        parameters.small_bearing_id,
                                        arm_clearance,
                                        parameters.small_bearing_shoulder_size,
                                        o_ring_minor_diameter,
                                        3 * parameters.extrusion_width,
                                        parameters.layer_height,
                                        parameters.small_screw_nut_s,
                                        parameters.small_screw_nut_height + parameters.small_screw_diameter / 6,
                                        True
                                        ).make_part("inner_road_wheel", ["3d_print"])
outer_road_wheel = road_wheel_generator(wheel_diameter,
                                        half_wheel_width,
                                        parameters.small_bearing_id,
                                        arm_clearance,
                                        parameters.small_bearing_shoulder_size,
                                        o_ring_minor_diameter,
                                        3 * parameters.extrusion_width,
                                        parameters.layer_height,
                                        parameters.small_screw_head_diameter,
                                        parameters.small_screw_head_height,
                                        False
                                        ).make_part("outer_road_wheel", ["3d_print"])
bogie = bogie_generator(bogie_wheel_spacing,
                        bogie_width, parameters.shoulder_screw_length + parameters.shoulder_screw_head_height,
                        parameters.small_bearing_od, parameters.small_bearing_thickness, parameters.small_bearing_shoulder_size,
                        4 * parameters.extrusion_width, 6 * parameters.extrusion_width,
                        12, # Pivot Z
                        wheel_diameter + 2 * clearance,
                        arm_thickness + 2 * arm_clearance,
                        arm_width,
                        100, # Arm cutout angle
                        parameters.shoulder_screw_diameter,
                        parameters.shoulder_screw_diameter2,
                        parameters.shoulder_screw_length,
                        parameters.shoulder_screw_screw_length,
                        parameters.shoulder_screw_head_diameter,
                        parameters.shoulder_screw_head_height,
                        parameters.shoulder_screw_nut_height,
                        parameters.shoulder_screw_nut_s,
                        parameters.overhang_angle,
                        ).make_part("bogie", ["3d_print"])

bogie_assembly = codecad.Assembly([bogie.translated_z(wheel_diameter / 2),
                                   inner_road_wheel.rotated_x(90).translated(bogie_wheel_spacing / 2,
                                                                             wheel_width / 2,
                                                                             wheel_diameter / 2),
                                   inner_road_wheel.rotated_x(90).translated(-bogie_wheel_spacing / 2,
                                                                             wheel_width / 2,
                                                                             wheel_diameter / 2),
                                   outer_road_wheel.rotated_x(-90).translated(bogie_wheel_spacing / 2,
                                                                              -wheel_width / 2,
                                                                              wheel_diameter / 2),
                                   outer_road_wheel.rotated_x(-90).translated(-bogie_wheel_spacing / 2,
                                                                              -wheel_width / 2,
                                                                              wheel_diameter / 2)]
                                 ).make_part("bogie_assembly")


def suspension_generator(params, state):
    arm_point = polar2rect(params[state] + params[SPRING_ARM_ANGLE_OFFSET], params[SPRING_ARM_LENGTH])
    anchor_point = (params[SPRING_ANCHOR_X], params[SPRING_ANCHOR_Y])

    v = (arm_point[0] - anchor_point[0], arm_point[1] - anchor_point[1])

    spring_length = math.hypot(v[0], v[1])
    spring_degrees = 90 - math.degrees(math.atan2(v[1], v[0]))
    spring = spring_placeholder_generator(spring_length).make_part("spring", ["vitamins"])

    degrees = -math.degrees(params[state])

    asm = codecad.Assembly([arm.rotated_x(90).rotated_y(degrees),
                            wheel.rotated_x(90).translated_x(params[ARM_LENGTH]).rotated_y(degrees),
                            spring.rotated_y(spring_degrees).translated(params[SPRING_ANCHOR_X], 15, params[SPRING_ANCHOR_Y])])

    return asm


if __name__ == "__main__":
    codecad.commandline_render(bogie.rotated_z(30).rotated_x(30), 0.1)
    sys.exit()

    print(params)
    width = suspension_width(params)
    height = suspension_height(params)

    print("width, height:", width, height)
    print("up travel:", (math.sin(params[ARM_UP_ANGLE]) - math.sin(params[ARM_NEUTRAL_ANGLE])) * params[ARM_LENGTH])
    print("down travel:", (math.sin(params[ARM_NEUTRAL_ANGLE]) - math.sin(params[ARM_DOWN_ANGLE])) * params[ARM_LENGTH])

    plot_wheel_forces(params)

    s1 = suspension_generator(params, ARM_DOWN_ANGLE).make_part("down_suspension")
    s2 = suspension_generator(params, ARM_NEUTRAL_ANGLE).make_part("neutral_suspension")
    s3 = suspension_generator(params, ARM_UP_ANGLE).make_part("up_suspension")

    o = codecad.Assembly([s1.translated_x(-width),
                          s2,
                          s3.translated_x(width)])

    codecad.commandline_render(o, 0.1)
