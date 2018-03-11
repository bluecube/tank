import math

import codecad
from codecad.shapes import *

import parameters
import suspension

nail_diameter = 1.25
nail_length = 16 # Including head

clearance = 1
thickness = nail_diameter + 2 * 3 * parameters.extrusion_width
segment_length = 15
width = 2 * nail_length
guide_height = 6
guide_length = 7
guide_width = suspension.wheel_gap - clearance
guide_side_angle = 30
guide_clearance_radius = 10
negative_bend_angle = 10

connector_width = 4

segment_width = width - 2 * (clearance + connector_width)
connector_length = 2 * thickness + clearance

assert suspension.wheel_width <= width + 2 * clearance

def track_segment_generator(thickness, length, width, connector_width,
                            pivot_diameter,
                            guide_height, guide_length, guide_width, guide_side_angle, guide_clearance_radius,
                            negative_bend_angle,
                            clearance):

    between_centers = length - thickness
    base = rectangle(width, length)
    connector_gap = rectangle(connector_width, 2 * thickness).offset(clearance / 2)
    base -= connector_gap.translated_y(length / 2)
    base -= connector_gap.translated_y(-length / 2)
    segment = base.extruded(thickness)

    # Round the sides to allow positive and negative bending
    c = math.cos(math.radians(negative_bend_angle))
    s = math.sin(math.radians(negative_bend_angle))
    cut = polygon2d([(-0.5 * s, 0.5 * c), (0.5, 0), (1, 0), (1, 1), (-1, 1)]) - \
                    circle(d=1) + \
                    half_plane().translated_y(0.5).rotated(negative_bend_angle)
    cut = cut.scaled(thickness)
    cut += circle(d=pivot_diameter * 0.92)
    cut = cut.translated_y(between_centers / 2)
    cut += cut.mirrored_y()
    segment -= cut.extruded(float("inf")).rotated_y(-90)

    guide_length = length - 2 * (thickness + clearance)

    # Make track guide
    guide = cylinder(r=guide_clearance_radius, h=guide_width).rotated_y(90)

    guide_side = half_space() \
                 .rotated_x(-guide_side_angle) \
                 .translated_y(-guide_width / 2) \
                 .translated_z(thickness / 2)

    guide = intersection([guide.translated_y(between_centers / 2),
                          guide.translated_y(-between_centers / 2),
                          rectangle(guide_width, guide_length).extruded(guide_height + thickness / 2, symmetrical=False),
                          guide_side.rotated_z(90),
                          guide_side.rotated_z(-90)])
    segment += guide

    # Make traction pattern
    #grooves = codecad.shapes.unsafe.Repetition(cylinder(r=groove_depth, h=float("inf"))
    #                                           .rotated_y(90)
    #                                           .translated_z(-thickness / 2),
    #                                           (None, length / 3, None)).translated_y(length / 6)
    #segment -= grooves

    return segment.rotated_z(90).translated_z(thickness / 2)


def track_connector_generator(thickness, length, width, pivot_diameter, clearance):
    between_centers = length - thickness
    connector = rectangle(thickness + clearance, 0).offset(thickness / 2)
    connector -= circle(d=pivot_diameter).translated_x(between_centers / 2)
    connector -= circle(d=pivot_diameter).translated_x(-between_centers / 2)
    return connector.extruded(width, symmetrical=False)


track_segment = track_segment_generator(thickness, segment_length, segment_width, connector_width,
                                        nail_diameter,
                                        guide_height, guide_length, guide_width, guide_side_angle, guide_clearance_radius,
                                        negative_bend_angle,
                                        clearance
                                        ).make_part("track_segment", ["3d_print"])
track_connector = track_connector_generator(thickness, connector_length, connector_width,
                                            nail_diameter, clearance
                                            ).make_part("track_connector", ["3d_print"])

def track_row(n):
    parts = []
    for i in range(n):
        dist = (i - n/2 + 0.5) * (segment_length + clearance)

        conn = track_connector \
            .rotated_x(90) \
            .translated(dist + (segment_length + clearance) / 2,
                        connector_width / 2, thickness / 2)

        parts.append(track_segment.translated_x(dist))
        parts.append(conn)
        parts.append(conn.translated_y((width - connector_width - clearance) / 2))
        parts.append(conn.translated_y(-(width - connector_width - clearance) / 2))

    return codecad.Assembly(parts).make_part("track_assembly")

if __name__ == "__main__":
    codecad.commandline_render(track_row(2).rotated_x(90), 0.1)
