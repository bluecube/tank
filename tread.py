import math

import codecad
from codecad.shapes import *

import parameters

def tread_segment_generator(thickness, length, width,
                            screw_diameter, screw_head_height,
                            guide_height, guide_length, guide_width, guide_side_angle,
                            negative_bend_angle,
                            groove_depth,
                            clearance):

    base_length = length + thickness + 2 * clearance
    space_for_fingers = width - 2 * screw_head_height - 4 * clearance
    finger_size_front = space_for_fingers / 2 / 3
    finger_size_back = space_for_fingers / 2 / 2
    finger_spacing = finger_size_front + finger_size_back + 2 * clearance

    base = rectangle(width, base_length)

    finger_space_front = rectangle(finger_size_back, 2 * thickness).offset(clearance)
    finger_space_front = codecad.shapes.unsafe.Repetition2D(finger_space_front,
                                                            (finger_spacing, None, None))
    finger_space_front = finger_space_front.translated(finger_spacing / 2, base_length / 2)

    finger_space_back = rectangle(finger_size_front, 2 * thickness).offset(clearance)
    finger_space_back = codecad.shapes.unsafe.Repetition2D(finger_space_back,
                                                           (finger_spacing, None, None))
    finger_space_back = finger_space_back.translated(0, -base_length / 2)

    # With just finger holes the screw head would collide with the back side of
    # next segment. We need to add an extra cutout for that.

    screw_head_cutout = rectangle(2 * finger_size_back,
                                  2 * thickness + 2 * clearance).translated_y(-base_length / 2)

    base -= union([finger_space_front, finger_space_back,
                  screw_head_cutout.translated_x(width / 2),
                  screw_head_cutout.translated_x(-width / 2)
                  ])
    segment = base.extruded(thickness)


    # Round the sides to allow positive and negative bending
    c = math.cos(math.radians(negative_bend_angle))
    s = math.sin(math.radians(negative_bend_angle))
    side_rounding = polygon2d([(-0.5 * s, 0.5 * c), (0.5, 0), (1, 0), (1, 1), (-1, 1)]) - \
                    circle(d=1) + \
                    half_plane().translated_y(0.5).rotated(negative_bend_angle)
    side_rounding = side_rounding.scaled(thickness)
    side_rounding = side_rounding.extruded(float("inf")).rotated_y(-90)
    side_rounding = side_rounding.translated_y(length/2)
    side_rounding += side_rounding.mirrored_y()
    segment -= side_rounding

    # Drill pin holes
    segment -= cylinder(h=float("inf"), d=screw_diameter * 0.9) \
               .rotated_y(90) \
               .translated_y(length / 2)
    segment -= cylinder(h=float("inf"), d=screw_diameter * 1.05) \
               .rotated_y(90) \
               .translated_y(-length / 2)

    # Make tread guide
    guide_radius = math.hypot(length / 2 + guide_length / 2, thickness / 2)
    guide = cylinder(r=guide_radius, h=guide_width).rotated_y(90)

    guide_side = half_space() \
                 .rotated_x(-guide_side_angle) \
                 .translated_y(-guide_width / 2) \
                 .translated_z(thickness / 2)

    guide = intersection([guide.translated_y(length / 2),
                          guide.translated_y(-length / 2),
                          rectangle(guide_width, guide_length).extruded(guide_height + thickness / 2, symmetrical=False),
                          guide_side.rotated_z(90),
                          guide_side.rotated_z(-90)])
    segment += guide

    # Make traction pattern
    grooves = codecad.shapes.unsafe.Repetition(cylinder(r=groove_depth, h=float("inf"))
                                               .rotated_y(90)
                                               .translated_z(-thickness / 2),
                                               (None, length / 3, None)).translated_y(length / 6)
    segment -= grooves

    return segment


tread_segment = tread_segment_generator(parameters.tread_thickness,
                                        parameters.tread_segment_length,
                                        parameters.tread_width,
                                        parameters.small_screw_diameter,
                                        parameters.small_screw_head_height,
                                        parameters.tread_guide_height,
                                        parameters.tread_guide_length,
                                        parameters.tread_guide_width,
                                        parameters.tread_guide_side_angle,
                                        parameters.tread_negative_bend_angle,
                                        parameters.tread_groove_depth,
                                        parameters.tread_clearance
                                        ).make_part("tread_segment", ["3d_print"])

def tread_segment_row(n):

if __name__ == "__main__":
    codecad.commandline_render(tread_segment.rotated_x(45), 0.1)
