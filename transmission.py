import math

import codecad
from codecad.shapes import *

import vitamins
import parameters
import tools
import drive_sprocket

transmission_steps = [(17, 39), (17, 39)]
    # Tooth counts of the transmission steps

motor_max_rpm = 930 * 11.6 # rpm, motor Kv * motor voltage

gear_tip_clearance = 1 # mm, distance between gear OD and neighboring geometry
module = 1.5

def shaft2_generator(sprocket_spline, gear_spline, spline_tolerance,
                     sprocket_bearing,
                     sprocket_spline_length,
                     sprocket_screw_diameter,
                     remaining_sprocket_screw_length,
                     sprocket_screw_nut_s,
                     sprocket_screw_nut_height,
                     hole_blinding_layer_height):
    h = 5
    shaft = gear_spline.offset(-spline_tolerance).extruded(h, symmetrical=False)

    cone_height = 10
    shaft += polygon2d([
        (0, 0),
        (0, cone_height),
        (sprocket_bearing.id / 2 + sprocket_bearing.shoulder_size, cone_height),
        (gear_spline.od / 2 - spline_tolerance, 0)]) \
        .revolved() \
        .rotated_x(90) \
        .translated_z(h)
    h += cone_height

    h += sprocket_bearing.thickness
    shaft += cylinder(d=sprocket_bearing.id, h=h, symmetrical=False)

    h += sprocket_spline_length
    shaft += sprocket_spline.offset(-spline_tolerance).extruded(h, symmetrical=False)

    # Sprocket screw and nuts holes
    sprocket_screw_nut_plane = h - remaining_sprocket_screw_length + sprocket_screw_diameter / 6 + sprocket_screw_nut_height
    shaft -= cylinder(d=sprocket_screw_diameter, h=2*remaining_sprocket_screw_length + sprocket_screw_diameter) \
        .translated_z(h)
    shaft -= regular_polygon2d(n=6, across_flats=sprocket_screw_nut_s) \
        .rotated(30) \
        .extruded(sprocket_screw_nut_height, symmetrical=False) \
        .translated_z(sprocket_screw_nut_plane - sprocket_screw_nut_height)
    shaft -= box(100, sprocket_screw_nut_s, sprocket_screw_nut_height) \
        .translated_x(50) \
        .translated_z(sprocket_screw_nut_plane - sprocket_screw_nut_height + sprocket_screw_nut_height / 2)

    if hole_blinding_layer_height:
        shaft += box(sprocket_screw_diameter, sprocket_screw_diameter, hole_blinding_layer_height) \
            .translated_z(sprocket_screw_nut_plane + hole_blinding_layer_height / 2)

    shaft.length = h

    return shaft

gear_shaft2_spline = tools.spline(20)
shaft2 = shaft2_generator(drive_sprocket.spline, gear_shaft2_spline, 0.1,
                         vitamins.large_bearing,
                         drive_sprocket.base.thickness,
                         vitamins.long_screw.diameter,
                         vitamins.long_screw.length - drive_sprocket.center_screw_wall_thickness,
                         vitamins.long_screw.lock_nut.s,
                         vitamins.long_screw.lock_nut.height,
                         parameters.overhang_hole_blinding) \
    .make_part("transmission_shaft2", ["3d_print"])

shaft2_assembly = codecad.assembly("shaft2_assembly",
                                   [drive_sprocket.drive_sprocket_assembly,
                                    shaft2.rotated_x(180).translated_z(shaft2.part.data.length + 20),
                                    vitamins.long_screw,
                                    vitamins.long_screw.lock_nut])

if __name__ == "__main__":
    ratio = 1
    for t1, t2 in transmission_steps:
        assert math.gcd(t1, t2) == 1
        ratio *= t2 / t1

    speed = (motor_max_rpm / 60) * drive_sprocket.circumference / ratio
    print("transmission ratios",
          ", ".join("{}:{}".format(*teeth) for teeth in transmission_steps),
          "({:.1f}:1) -> {:.1f}m/s, {:.1f}km/h".format(ratio, speed/1000, 3.6 * speed / 1000))

    codecad.commandline_render((shaft2_assembly.shape().rotated_y(-90) & half_space()).rotated_z(-30).rotated_x(15))
