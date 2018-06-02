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

# The layout calculations and tensioner assembly are referenced to the following
# suspension module's main pivot

wheel_diameter = 55
wheel_base_y = 0 # Approximate y coordinate of the wheel center
spring_anchor_y = 0
spring_angle_offset = 70 # degrees
spring_inversion_safety_distance = 2

wheel_screw = vitamins.long_screw # Screw used as an axle for the tensioner wheel

wheel_clearance = suspension.wheel_clearance
arm_clearance = suspension.arm_clearance
track_clearance = track.body_clearance

arm_width = suspension.arm_width
arm_thickness = suspension.arm_thickness
arm_pivot_thickness = suspension.arm_pivot_thickness
arm_length = wheel_diameter / 2 - arm_pivot_thickness / 2 - wheel_clearance
    # Max length so that the pivot is still completely hidden by the wheel

spring_anchor_point = Vector(0 - suspension.arm_pivot_thickness / 2 - vitamins.spring.top_mount_od / 2 - suspension.arm_clearance,
                             spring_anchor_y)

def spring_arm_length_equation(spring_arm_length):
    return arm_length**2 + spring_arm_length**2 - 2 * arm_length * spring_arm_length * math.cos(math.radians(spring_angle_offset)) - \
           (wheel_diameter / 2 + wheel_clearance + vitamins.spring.bottom_mount_od / 2)**2
spring_arm_length = scipy.optimize.brentq(spring_arm_length_equation, 0, 2 * wheel_diameter)

pivot_to_spring_base = spring_arm_length + vitamins.spring.length - vitamins.spring.travel - spring_inversion_safety_distance
pivot_y = wheel_base_y + arm_length
pivot_position = Vector(-math.sqrt(pivot_to_spring_base**2 - pivot_y**2), pivot_y)
pivot_to_spring_base_angle = -math.degrees(math.asin(pivot_y / pivot_to_spring_base))

def get_arm_angle(spring_length):
    return pivot_to_spring_base_angle - \
           math.degrees(math.acos((spring_arm_length**2 + pivot_to_spring_base**2 - spring_length**2) /
                                  (2 * spring_arm_length * pivot_to_spring_base))) - \
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
        half -= rectangle(10, 2*(4.5-wheel_clearance)).offset(wheel_clearance).translated(arm_length, -arm_clearance)

    half = half \
        .revolved() \
        .rotated_x(90)

    if parameters.overhang_hole_blinding:
        half += cylinder(r=mid_radius, h=parameters.overhang_hole_blinding) \
            .translated_z(bearing_height + bearing.thickness)

    half -= tools.crown_cutout(outer_diameter=2 * mid_radius,
                               inner_diameter=inner_radius + mid_radius,
                               tolerance=crown_tolerance,
                               height=total_height - cone_height,
                               inverse=inner_half) \
        .translated_z(total_height)

    #half -= tools.wheel_lightening_holes(5,
    #                                     bearing.od / 2 + wall_thickness,
    #                                     radius - wall_thickness,
    #                                     wall_thickness)

    return half


def arm_generator(thickness, pivot_thickness, width,
                  arm_length, spring_arm_length, spring_angle_offset,
                  cone_height,
                  wheel_bearing):

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
        .extruded(width, symmetrical=False)
    cone = tools.cone(height=cone_height,
                      upper_diameter=cone_upper_diameter,
                      lower_diameter=(cone_upper_diameter + thickness) / 2 + cone_height,
                      base_height=width) \
        .translated_x(arm_length)
    cone &= outline.extruded(float("inf"))

    arm += cone

    return arm

def preview(arm_angle):
    wheel_position = pivot_position + Vector.polar(arm_length, arm_angle)

    o = circle(d=wheel_diameter) - circle(d=0.9 * wheel_diameter) + circle(d=2)
    o = o.translated(wheel_position.x, wheel_position.y)

    o += circle(d=5).translated(pivot_position.x, pivot_position.y)
    spring_point = pivot_position + Vector.polar(spring_arm_length, arm_angle + spring_angle_offset)
    o += capsule(spring_anchor_point.x, spring_anchor_point.y, spring_point.x, spring_point.y, vitamins.spring.diameter)
    return o

#o = capsule(worst_track_guide_points[0][0], worst_track_guide_points[0][1],
#            worst_track_guide_points[1][0], worst_track_guide_points[1][1],
#            1)
#o += circle(d=vitamins.spring.top_mount_od).translated(spring_anchor_point.x, spring_anchor_point.y)
#o += circle(d=vitamins.spring.top_mount_od).translated(pivot_x, pivot_y)
#o = o.extruded(1).rotated_x(90)

#o = o.make_part("X")

#asm = codecad.assembly("tensioner_test", [o, suspension.suspension_generator(False, suspension.arm_up_angle),
                                          #o.translated_z(-100), suspension.suspension_generator(False).translated_z(-100),
                                          #o.translated_z(-200), suspension.suspension_generator(False, suspension.arm_down_angle).translated_z(-200)])

o = preview(arm_angle_front).translated_y(-50) + preview(arm_angle_back).translated_y(50)

inner_wheel_half = wheel_generator(wheel_diameter, suspension.wheel_width,
                                   0.1, # Crown tolerance
                                   vitamins.small_bearing,
                                   7, # bearing_height
                                   4 * parameters.extrusion_width,
                                   track.guide_height, track.guide_width, track.guide_side_angle, track.clearance,
                                   True) \
    .make_part("inner_tensioner_wheel", ["3d_print"])
outer_wheel_half = wheel_generator(wheel_diameter, suspension.wheel_width,
                                   0.1, # Crown tolerance
                                   vitamins.small_bearing,
                                   suspension.wheel_width + wheel_clearance + arm_width - wheel_screw.length,
                                   4 * parameters.extrusion_width,
                                   track.guide_height, track.guide_width, track.guide_side_angle, track.clearance,
                                   False) \
    .make_part("outer_tensioner_wheel", ["3d_print"])
wheel_assembly = codecad.assembly("tensioner_wheel_assembly",
                                  [inner_wheel_half.rotated_x(90).translated_y(suspension.wheel_width / 2),
                                   outer_wheel_half.rotated_x(-90).translated_y(-suspension.wheel_width / 2),
                                   wheel_screw] +
                                  [vitamins.small_bearing] * 2)

arm = arm_generator(arm_thickness, arm_pivot_thickness, arm_width,
                  arm_length, spring_arm_length, spring_angle_offset,
                  9,
                  vitamins.small_bearing)


if __name__ == "__main__":
    def p(name, f=lambda x: x):
        print(name, f(globals()[name]))

    p("spring_anchor_point")
    p("arm_length")
    p("spring_arm_length")
    p("pivot_position")
    p("arm_angle_back")
    p("wheel_position_back")
    p("arm_angle_front")
    p("wheel_position_front")
    p("pivot_to_spring_base_angle")
    p("wheel_travel")
    p("wheel_x_travel")

    codecad.commandline_render(wheel_assembly.rotated_x(90).shape() & half_space())
