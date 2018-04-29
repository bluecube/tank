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

wheel_max_y = 10

wheel_track_guide_radius = suspension.wheel_diameter / 2 - track.guide_height
worst_wheel_position = Vector(math.cos(suspension.arm_up_angle),
                                   math.sin(suspension.arm_up_angle)) * suspension.arm_length + \
                            suspension.get_bogie_wheel_position(suspension.arm_up_angle - suspension.arm_neutral_angle - suspension.bogie_swing_angle,
                                                                 -1)
worst_wheel_spring_point = suspension.spring_up_point

worst_track_guide_points = util.outer_tangent(worst_wheel_spring_point, suspension.arm_thickness / 2 + track.body_clearance,
                                              worst_wheel_position, wheel_track_guide_radius)

def wheel_back_center_equation(x):
    dist = util.point_to_line_distance(Vector(x, wheel_max_y),
                                       worst_track_guide_points[1],
                                       worst_track_guide_points[0])
    return dist - wheel_track_guide_radius

wheel_back_x = scipy.optimize.brentq(wheel_back_center_equation,
                                     -100 * suspension.wheel_diameter,
                                     100 * suspension.wheel_diameter)
print(wheel_back_x)
wheel_back_x = min(wheel_back_x,
                   -math.sqrt((suspension.wheel_diameter / 2 +
                               suspension.arm_pivot_thickness / 2 +
                               suspension.wheel_clearance)**2 -
                              wheel_max_y**2))
print(wheel_back_x)

worst_track_guide_points2 = util.outer_tangent(Vector(wheel_back_x, wheel_max_y),
                                               wheel_track_guide_radius,
                                               worst_wheel_position,
                                               wheel_track_guide_radius)

spring_anchor_point = Vector(suspension.spring_down_point.x -
                             (vitamins.spring.diameter - vitamins.spring.top_mount_thickness) / 2 - suspension.arm_clearance -
                             vitamins.spring.top_mount_od / 2 -
                             suspension.arm_thickness -
                             suspension.arm_clearance,
                             -10)

o = circle(d=suspension.wheel_diameter) \
    .translated(wheel_back_x, wheel_max_y)
o += capsule(worst_track_guide_points[0][0], worst_track_guide_points[0][1],
             worst_track_guide_points[1][0], worst_track_guide_points[1][1],
             1)
o += capsule(worst_track_guide_points2[0][0], worst_track_guide_points2[0][1],
             worst_track_guide_points2[1][0], worst_track_guide_points2[1][1],
             2)
#o += circle(d=suspension.wheel_diameter) \
#    .translated(worst_wheel_position[0], worst_wheel_position[1])
o = o.extruded(1).rotated_x(90)

o = o.make_part("X")

asm = codecad.assembly("tensioner_test", [o, suspension.suspension_assembly_left])

if __name__ == "__main__":
    def p(name, f=lambda x: x):
        print(name, f(globals()[name]))

    p("spring_anchor_point")
    p("worst_wheel_position")
    p("worst_wheel_spring_point")
    p("worst_track_guide_points")
    p("wheel_back_x")
    p("wheel_max_y")

    codecad.commandline_render(asm.rotated_z(10))
