import math

import scipy.optimize

import codecad
from codecad.shapes import *
from codecad.util import Vector

import util
import tools
import vitamins
import parameters
import suspension
import track

from parameters import wheel_clearance

wheel_diameter = 55
wheel_base_y = 0 # Approximate y coordinate of the wheel center
spring_anchor_y = 15
spring_angle_offset = 90 # degrees
spring_inversion_safety_distance = 4

pivot_round_clearance = suspension.pivot_round_clearance

wheel_screw = vitamins.m3x35_screw # Screw used as an axle for the tensioner wheel
spring_screw = vitamins._m3_screw(16) # TODO: Decide on the correct screw to use here
wheel_bearing = vitamins.small_bearing

arm_clearance = suspension.arm_clearance
track_clearance = track.body_clearance

arm_width = suspension.arm_width
arm_thickness = suspension.arm_thickness

inner_bearing_height = 7 # How deep is the bearing seated in the inne wheel half
arm_length = (wheel_diameter / 2 + wheel_bearing.od / 2 + inner_bearing_height) / 2
    # Max length so that the pivot is still completely hidden by the wheel
arm_pivot_thickness = 2 * (wheel_diameter / 2 - arm_length - wheel_clearance)

def spring_arm_length_equation(spring_arm_length):
    return arm_length**2 + spring_arm_length**2 - 2 * arm_length * spring_arm_length * math.cos(math.radians(spring_angle_offset)) - \
           (wheel_diameter / 2 + wheel_clearance + vitamins.spring.bottom_mount_od / 2)**2
spring_arm_length = scipy.optimize.brentq(spring_arm_length_equation, 0, 2 * wheel_diameter)

pivot_to_spring_anchor = spring_arm_length + vitamins.spring.length - vitamins.spring.travel - spring_inversion_safety_distance
pivot_y = wheel_base_y + arm_length
pivot_position = Vector(0, pivot_y)
pivot_to_spring_anchor_angle = -math.degrees(math.asin((pivot_y - spring_anchor_y) / pivot_to_spring_anchor))

spring_anchor_point = Vector(math.sqrt(pivot_to_spring_anchor**2 - (pivot_y - spring_anchor_y)**2), spring_anchor_y)
to_suspension_pivot = spring_anchor_point.x + suspension.arm_pivot_thickness / 2 + vitamins.spring.top_mount_od / 2 + suspension.arm_clearance

def get_arm_angle(spring_length):
    return pivot_to_spring_anchor_angle - \
           math.degrees(math.acos((spring_arm_length**2 + pivot_to_spring_anchor**2 - spring_length**2) /
                                  (2 * spring_arm_length * pivot_to_spring_anchor))) - \
           spring_angle_offset

arm_angle_back = get_arm_angle(vitamins.spring.length - vitamins.spring.travel)
arm_angle_front = get_arm_angle(vitamins.spring.length)

wheel_position_back = pivot_position + Vector.polar(arm_length, arm_angle_back)
wheel_position_front = pivot_position + Vector.polar(arm_length, arm_angle_front)

wheel_travel = abs(wheel_position_back - wheel_position_front)
wheel_x_travel = abs(wheel_position_back.x - wheel_position_front.x)

def wheel_generator(diameter, whole_width,
                    crown_tolerance,
                    bearing,
                    bearing_height,
                    wall_thickness,
                    guide_height, guide_width, guide_side_angle, track_clearance,
                    arm_length, screw_head_diameter, screw_head_height, screw_head_clearance,
                    inner_half):
    radius = diameter / 2
    mid_radius = radius - guide_height - track_clearance
    inner_radius = bearing.od / 2 - bearing.shoulder_size

    cylinder_height = (whole_width - guide_width - track_clearance) / 2
    cone_height = cylinder_height + math.tan(math.radians(guide_side_angle)) * (radius - mid_radius)
    total_height = whole_width - cone_height

    assert bearing_height + bearing.thickness < total_height

    half = polygon2d([(radius, 0),
                      (radius, cylinder_height),
                      (mid_radius, cone_height),
                      (mid_radius, total_height),
                      (inner_radius, total_height),
                      (inner_radius, bearing_height + bearing.thickness),
                      (bearing.od / 2, bearing_height + bearing.thickness),
                      (bearing.od / 2, bearing_height),
                      (bearing.od / 2 + bearing_height, 0)])

    if inner_half:
        half -= rectangle(screw_head_diameter,
                          2 * screw_head_height) \
            .offset(screw_head_clearance) \
            .translated_x(arm_length)

    half = half \
        .revolved() \
        .rotated_x(90)

    lightening_holes = 5

    half -= unsafe.CircularRepetition(cylinder(d=screw_head_diameter * 1.02,
                                               h=float("inf")) \
                                        .translated_x(arm_length),
                                      lightening_holes)

    if parameters.overhang_hole_blinding:
        half += cylinder(r=bearing.od, h=parameters.overhang_hole_blinding) \
            .translated_z(bearing_height + bearing.thickness)

        if inner_half:
            plug = circle(r=arm_length + screw_head_diameter * 0.55) - circle(r=arm_length - screw_head_diameter * 0.55)
            half += plug \
                .extruded(parameters.overhang_hole_blinding, symmetrical=False) \
                .translated_z(screw_head_height + screw_head_clearance)

    if inner_half and parameters.overhang_spokes_width and parameters.overhang_spokes_height:
        half += unsafe.CircularRepetition2D(rectangle(screw_head_diameter + 2 * screw_head_clearance,
                                                      parameters.overhang_spokes_width).translated_x(arm_length),
                                            lightening_holes) \
            .extruded(2 * parameters.overhang_spokes_height) \
            .translated_z(screw_head_height + screw_head_clearance) \
            .rotated_z(180 / lightening_holes)

    half -= tools.crown_cutout(outer_diameter=2 * mid_radius,
                               inner_diameter=inner_radius + mid_radius,
                               tolerance=crown_tolerance,
                               height=total_height - cone_height,
                               inverse=inner_half) \
        .translated_z(total_height)

    return half


def arm_generator(thickness, pivot_thickness, width, spring_point_width,
                  arm_length, spring_arm_length, spring_angle_offset,
                  cone_height,
                  wheel_diameter, wheel_clearance,
                  pivot_hole_diameter,
                  wheel_bearing, wheel_screw,
                  spring_screw):

    cone_upper_diameter = wheel_bearing.id + 2 * wheel_bearing.shoulder_size

    spring_point = Vector.polar(spring_arm_length, spring_angle_offset)

    p1, _ = util.outer_tangent(Vector(0, 0), (pivot_thickness - thickness) / 2,
                               Vector(arm_length, 0), 0)
    _, p2 = util.outer_tangent(spring_point, 0,
                               Vector(0, 0), (pivot_thickness - thickness) / 2)

    outline = polygon2d([(p1.x, p1.y),
                         (arm_length, 0),
                         (spring_point.x, spring_point.y),
                         (p2.x, p2.y)]) \
        .offset(thickness / 2)
    outline += circle(d=pivot_thickness)

    arm = outline \
        .extruded(spring_point_width, symmetrical=False)

    arm -= rectangle(wheel_diameter, 2 * spring_point_width) \
        .translated_y(spring_point_width + arm_width + wheel_clearance) \
        .offset(wheel_clearance) \
        .revolved() \
        .rotated_x(90) \
        .translated_x(arm_length)

    cone = tools.cone(height=cone_height,
                      upper_diameter=cone_upper_diameter,
                      lower_diameter=(cone_upper_diameter + thickness) / 2 + cone_height,
                      base_height=width) \
        .translated_x(arm_length)
    arm += cone & outline.extruded(float("inf"))


    arm -= cylinder(d=pivot_hole_diameter, h=float("inf"))

    arm -= tools.screw_hole_with_nut_pocket(wheel_screw) \
        .translated_x(arm_length)

    arm -= tools.screw_hole_with_nut_pocket(spring_screw) \
        .translated(*spring_point)

    return arm

inner_wheel_half = wheel_generator(wheel_diameter, suspension.wheel_width,
                                   0.1, # Crown tolerance
                                   wheel_bearing,
                                   inner_bearing_height,
                                   4 * parameters.extrusion_width,
                                   track.guide_height, track.guide_width, track.guide_side_angle, track.clearance,
                                   arm_length, vitamins.shoulder_screw.head_diameter, vitamins.shoulder_screw.head_height - wheel_clearance, wheel_clearance,
                                   True) \
    .make_part("inner_tensioner_wheel", ["3d_print"])
outer_wheel_half = wheel_generator(wheel_diameter, suspension.wheel_width,
                                   0.1, # Crown tolerance
                                   wheel_bearing,
                                   suspension.wheel_width + wheel_clearance + arm_width - wheel_screw.length,
                                   4 * parameters.extrusion_width,
                                   track.guide_height, track.guide_width, track.guide_side_angle, track.clearance,
                                   arm_length, vitamins.shoulder_screw.head_diameter, vitamins.shoulder_screw.head_height - wheel_clearance, wheel_clearance,
                                   False) \
    .make_part("outer_tensioner_wheel", ["3d_print"])
arm_left = arm_generator(arm_thickness, arm_pivot_thickness, arm_width, spring_screw.length - vitamins.spring.bottom_mount_thickness,
                         arm_length, spring_arm_length, spring_angle_offset,
                         inner_bearing_height + wheel_clearance, # cone height
                         wheel_diameter, wheel_clearance,
                         vitamins.shoulder_screw.diameter2 + pivot_round_clearance,
                         wheel_bearing, wheel_screw,
                         spring_screw) \
    .make_part("tensioner_arm_left", ["3d_print"])
arm_right = arm_left.shape().mirrored_x().make_part("tensioner_arm_right", ["3d_print"])

wheel_assembly = codecad.assembly("tensioner_wheel_assembly",
                                  [inner_wheel_half.rotated_x(90),
                                   outer_wheel_half.rotated_x(-90).translated_y(-suspension.wheel_width)] +
                                  [wheel_bearing] * 2)

def tensioner_generator(right, arm_angle = (arm_angle_back + arm_angle_front) / 2):
    spring_point = Vector.polar(spring_arm_length, arm_angle + spring_angle_offset) + pivot_position

    v = spring_point.flattened() - spring_anchor_point.flattened()
    length = abs(v)

    spring_degrees = 90 - math.degrees(math.atan2(v.y, v.x))
    spring = vitamins.spring(length)

    if right:
        arm = arm_right
        multiplier = 1
    else:
        arm = arm_left.rotated_y(180)
        multiplier = -1

    arm_angle = -arm_angle

    asm = codecad.assembly("tensioner_assembly_" + ("right" if right else "left"),
                           [arm \
                                .rotated_x(-90) \
                                .rotated_y(180) \
                                .rotated_y(arm_angle) \
                                .translated(pivot_position.x, pivot_position.z, pivot_position.y),
                            wheel_assembly \
                                .translated_y(-arm_width - wheel_clearance) \
                                .rotated_z(90 + 90 * multiplier) \
                                .translated_x(arm_length) \
                                .rotated_y(arm_angle) \
                                .translated(pivot_position.x, pivot_position.z, pivot_position.y),
                            spring \
                                .rotated_y(spring_degrees) \
                                .translated(spring_anchor_point.x,
                                            multiplier * (spring_anchor_point.z + vitamins.spring.top_mount_thickness / 2),
                                            spring_anchor_point.y),
                            wheel_screw,
                            wheel_screw.lock_nut,
                            spring_screw,
                            spring_screw.lock_nut,
                            vitamins.shoulder_screw,
                            vitamins.shoulder_screw.lock_nut,
                            vitamins.m5x20_screw,
                            vitamins.m5x20_screw.lock_nut])

    return asm


tensioner_assembly_left = tensioner_generator(False)
tensioner_assembly_right = tensioner_generator(True)

if __name__ == "__main__":
    def p(name, f=lambda x: x):
        print(name, f(globals()[name]))

    p("to_suspension_pivot")
    p("spring_anchor_point")
    p("arm_length")
    p("spring_arm_length")
    p("pivot_position")
    p("arm_angle_back")
    p("wheel_position_back")
    p("arm_angle_front")
    p("wheel_position_front")
    p("pivot_to_spring_anchor_angle")
    p("wheel_travel")
    p("wheel_x_travel")

    codecad.commandline_render(tensioner_generator(False, arm_angle_back).shape().rotated_z(180).translated_x(150)
                               + tensioner_generator(False, arm_angle_front).shape().rotated_z(180).translated_z(70).translated_x(150)
                               + tensioner_generator(False, arm_angle_back).shape()
                               + tensioner_generator(False, arm_angle_front).shape().translated_z(70))
    #codecad.commandline_render(wheel_assembly)
