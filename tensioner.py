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

wheel_diameter = 50
wheel_base_y = 0 # Approximate y coordinate of the wheel center
spring_anchor_y = 0
spring_angle_offset = 70 # degrees
spring_inversion_safety_distance = 2

wheel_clearance = suspension.wheel_clearance
arm_clearance = suspension.arm_clearance
track_clearance = track.body_clearance

arm_thickness = suspension.arm_thickness
arm_length = wheel_diameter / 2 - arm_thickness / 2 - wheel_clearance
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

    codecad.commandline_render(o)
