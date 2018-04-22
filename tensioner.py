import math

import scipy.optimize

import codecad
from codecad.shapes import *
from codecad.util import Vector

import tools
import vitamins
import parameters
import suspension
import track

# The layout calculations and tensioner assembly are referenced to the following
# suspension module's main pivot

road_wheel_track_guide_radius = suspension.wheel_diameter / 2 - track.guide_height
worst_road_wheel_position = Vector(math.cos(suspension.arm_up_angle),
                                    math.sin(suspension.arm_up_angle)) * suspension.arm_length + \
                             suspension.get_bogie_wheel_position(suspension.arm_up_angle - suspension.arm_neutral_angle - suspension.bogie_swing_angle,
                                                                 -1)
worst_road_wheel_spring_point = suspension.spring_up_point

worst_track_guide_line


spring_anchor_point = Vector(suspension.spring_down_point.x -
                             (vitamins.spring.diameter - vitamins.spring.top_mount_thickness) / 2 - suspension.arm_clearance -
                             vitamins.spring.top_mount_od / 2 -
                             suspension.arm_thickness -
                             suspension.arm_clearance,
                             -10)

if __name__ == "__main__":
    def p(name, f=lambda x: x):
        print(name, f(globals()[name]))

    p("spring_anchor_point")
    p("worst_drive_wheel_position")
    p("worst_drive_wheel_spring_point")
