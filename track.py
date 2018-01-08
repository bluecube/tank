import math

import codecad
from codecad.shapes import *

import parameters

nail_diameter = 1.5
nail_length = 35 # Including head
nail_head_diameter = 2
nail_head_length = 1.5

thickness = nail_diameter + 2 * 3 * parameters.extrusion_width
segment_length = 15
width = nail_length + nail_head_length # We want the segments to be symmetrical
guide_height = 4
guide_length = 7
guide_width = 8
guide_side_angle = 45
guide_clearance = 0.5
negative_bend_angle = 10
groove_depth = 1
clearance = 1


def track_segment_generator(thickness, length, width,
                            pivot_diameter, pivot_head_height,
                            guide_height, guide_length, guide_width, guide_side_angle,
                            negative_bend_angle,
                            groove_depth,
                            clearance):

    base_length = length + thickness + 2 * clearance
    space_for_fingers = width - 2 * pivot_head_height - 4 * clearance
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

    # With just finger holes the pivot head would collide with the back side of
    # next segment. We need to add an extra cutout for that.

    pivot_head_cutout = rectangle(2 * finger_size_back,
                                  2 * thickness + 2 * clearance).translated_y(-base_length / 2)

    base -= union([finger_space_front, finger_space_back,
                   pivot_head_cutout.translated_x(width / 2),
                   pivot_head_cutout.translated_x(-width / 2)
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
    segment -= cylinder(h=float("inf"), d=pivot_diameter * 0.92) \
               .rotated_y(90) \
               .translated_y(length / 2)
    segment -= cylinder(h=float("inf"), d=pivot_diameter) \
               .rotated_y(90) \
               .translated_y(-length / 2)

    # Make track guide
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

    return segment.rotated_z(90)


track_segment = track_segment_generator(thickness,
                                        segment_length,
                                        width,
                                        nail_diameter,
                                        nail_head_length,
                                        guide_height,
                                        guide_length,
                                        guide_width,
                                        guide_side_angle,
                                        negative_bend_angle,
                                        groove_depth,
                                        clearance
                                        ).make_part("track_segment", ["3d_print"])

def track_row(n):
    return codecad.Assembly([track_segment \
                                .translated_x((i - n/2 + 0.5) * segment_length)
                             for i in range(n)]).make_part("track_assembly")

if __name__ == "__main__":
    codecad.commandline_render(track_row(10), 0.1)
