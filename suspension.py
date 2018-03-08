import sys
import math

import scipy.optimize

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

wheel_diameter = 30
wheel_width = 30 # Total width of the wheel pair
arm_width = 8

bogie_spacing = 110 # [mm] distance between bogies
bogie_wheel_spacing = 50 # [mm] distance between wheels of one bogie
bogie_width = arm_width
bogie_pivot_z = 7
bogie_arm_cutout_angle = 60 # Degrees

suspension_travel = 30 # [mm]
suspension_sag = 0.3 # Ratio of travel from neutral position down

half_wheel_width = (wheel_width - bogie_width) / 2 - arm_clearance

assert wheel_width - 2 * half_wheel_width >= track.guide_width + track.clearance, \
    "The track guide has enough space between the wheel halves"
assert wheel_width <= track.width

arm_thickness = parameters.shoulder_screw_diameter2 + 12 * parameters.extrusion_width

def get_spring_anchor_point(spring_arm_length):
    """ Return the spring anchor point coordinates in 2D relative to arm pivot as codecad Vector.
    Spring is placed to be horizontal and right angle to the arm at full compression. """
    return codecad.util.Vector(spring_length - spring_travel, -spring_arm_length)

def get_travel_angle(spring_arm_length, spring_anchor_point):
    """ Calculate travel angle of the suspension arm based on spring length.
    Uses spring anchor point placement from get_spring_anchor_point. """
    spring_compressed_length = spring_length - spring_travel

    if spring_arm_length == 0:
        return 0

    spring_anchor_distance = abs(spring_anchor_point)
    compressed_anchor_angle = math.atan(spring_compressed_length / spring_arm_length)

    tmp = (spring_arm_length**2 + spring_anchor_distance**2 - spring_length**2) / \
          (2 * spring_arm_length * spring_anchor_distance)
    return math.acos(tmp) - compressed_anchor_angle

def get_spring_point(spring_arm_length, angle):
    return codecad.util.Vector(-math.sin(angle), -math.cos(angle)) * spring_arm_length

def spring_arm_length_equation(spring_arm_length):
    """ Equation describing spring location relative to the pivot. """
    spring_anchor_point = get_spring_anchor_point(spring_arm_length)
    travel_angle = get_travel_angle(spring_arm_length, spring_anchor_point)
    spring_down_position_point = get_spring_point(spring_arm_length, travel_angle)

    u = (spring_anchor_point - spring_down_position_point).normalized()
    v = codecad.util.Vector(u.y, -u.x)

    spring_axis_to_pivot_point = spring_anchor_point.dot(v)

    return spring_axis_to_pivot_point - (arm_thickness / 2 + spring_diameter / 2 + arm_clearance)

spring_arm_length = scipy.optimize.brentq(spring_arm_length_equation, 0, arm_thickness + spring_diameter)
spring_anchor_point = get_spring_anchor_point(spring_arm_length)
travel_angle = get_travel_angle(spring_arm_length, spring_anchor_point)

# The highest distance above bogie pivot where the wheel might interfere
wheel_above_bogie_pivot = wheel_diameter / 2 + \
    math.sin(math.radians(bogie_arm_cutout_angle / 2)) * bogie_wheel_spacing / 2 - \
    math.cos(math.radians(bogie_arm_cutout_angle / 2)) * bogie_pivot_z
bogie_pivot_upper_limit = spring_anchor_point.y - spring_diameter / 2 - clearance - wheel_above_bogie_pivot

def get_arm_angle(arm_length, y):
    """ Calculate angle of the suspension arm at the top position """
    return math.asin(y / arm_length)

def get_arm_travel(arm_length, up_arm_angle):
    """ Calculate total length wheel vertical travel """
    return bogie_pivot_upper_limit - math.sin(up_arm_angle - travel_angle) * arm_length

def arm_length_equation(arm_length):
    up_angle = get_arm_angle(arm_length, bogie_pivot_upper_limit)
    down_angle = up_angle - travel_angle
    assert down_angle > -math.pi / 2

    travel = get_arm_travel(arm_length, up_angle)
    assert travel >= suspension_travel

    neutral_angle = get_arm_angle(arm_length, bogie_pivot_upper_limit - (1 - suspension_sag) * travel)

    neutral_spring_length = abs(spring_anchor_point - get_spring_point(spring_arm_length, up_angle - neutral_angle))
    neutral_spring_force = spring_preload_force + (spring_full_compression_force - spring_preload_force) * (spring_length - neutral_spring_length) / spring_travel
    return neutral_spring_force
    #neutral_spring_torque =

    #neutral_wheel_torque = parameters.design_weight * arm_length * math.cos(neutral_angle)

    #return neutral_wheel_torque - neutral_spring_torque

print(arm_length_equation(100))

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
                    shoulder_screw_nut_s):

    assert arm_cutout_angle < 180

    bearing_radius = bearing_diameter / 2
    nut_outer_diameter = shoulder_screw_nut_s * 2 / math.sqrt(3)
    pivot_protected_diameter = max(shoulder_screw_diameter2, nut_outer_diameter) + 2 * thin_wall
    pivot_end_diameter = pivot_protected_diameter + 2 * thick_wall
    pivot_to_wheel_distance = math.hypot(wheel_spacing / 2, pivot_z)
    wheel_cutout_angled_part = min(pivot_to_wheel_distance - wheel_cutout_diameter / 2 - pivot_protected_diameter / 2, (upper_thickness - lower_thickness) / 2)

    assert pivot_to_wheel_distance >= thin_wall + wheel_cutout_diameter / 2 + arm_cutout_diameter / 2

    bogie = polygon2d([(-wheel_spacing / 2, 0),
                       (wheel_spacing / 2, 0),
                       (0, pivot_z)]) \
        .offset((bearing_diameter + thin_wall + thick_wall) / 2) \
        .extruded(upper_thickness) \
        .rotated_x(90) \
        .translated_z((thick_wall - thin_wall) / 2)

    bottom_z = -bearing_radius - thin_wall

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
    cutout = polygon2d([(-bearing_radius, -bearing_thickness),
                        (bearing_radius, -bearing_thickness),
                        (bearing_radius, 0),
                        (wheel_cutout_diameter / 2, 0),
                        (wheel_cutout_diameter / 2, (upper_thickness - lower_thickness) / 2 - wheel_cutout_angled_part),
                        (wheel_cutout_diameter / 2 + (upper_thickness - lower_thickness) / 2 + wheel_cutout_angled_part, upper_thickness - lower_thickness),
                        (-bearing_radius, upper_thickness - lower_thickness)]) \
        .revolved()
    for x in [-1, 1]:
        x = x * wheel_spacing / 2
        bogie -= cylinder(d=bearing_diameter - 2 * bearing_shoulder_size, h=lower_thickness).rotated_x(90).translated_x(x)
        for y in [-1, 1]:
            bogie -= cutout.rotated_z(90 - y * 90).translated(x, y * lower_thickness / 2, 0)

    # bottom lightening angles
    base_thickness = upper_thickness / 2 - (pivot_z - bottom_z - pivot_protected_diameter / 2)
    for y in [-1, 1]:
        bogie -= half_space().rotated_x(90 + y * 135).translated(0, -y * base_thickness, bottom_z)

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
                        bogie_pivot_z,
                        wheel_diameter + 2 * clearance,
                        arm_thickness + 2 * arm_clearance,
                        arm_width,
                        bogie_arm_cutout_angle,
                        parameters.shoulder_screw_diameter,
                        parameters.shoulder_screw_diameter2,
                        parameters.shoulder_screw_length,
                        parameters.shoulder_screw_screw_length,
                        parameters.shoulder_screw_head_diameter,
                        parameters.shoulder_screw_head_height,
                        parameters.shoulder_screw_nut_height,
                        parameters.shoulder_screw_nut_s,
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
    codecad.commandline_render(bogie.rotated_z(45).rotated_x(0), 0.1)
    sys.exit()

    plot_wheel_forces(params)

    s1 = suspension_generator(params, ARM_DOWN_ANGLE).make_part("down_suspension")
    s2 = suspension_generator(params, ARM_NEUTRAL_ANGLE).make_part("neutral_suspension")
    s3 = suspension_generator(params, ARM_UP_ANGLE).make_part("up_suspension")

    o = codecad.Assembly([s1.translated_x(-width),
                          s2,
                          s3.translated_x(width)])

    codecad.commandline_render(o, 0.1)
