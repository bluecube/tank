import math
import itertools

import codecad
from codecad.shapes import *

import tools
import vitamins
import parameters
import suspension

vitamins.nail.diameter = 1.25
vitamins.nail.length = 16

clearance = 1
segment_pin_clearance = 0.05
connector_pin_clearance = 0.25

wall = 3 * parameters.extrusion_width

segment_length = 10 # Length between centers
width = 2 * vitamins.nail.length + 1
guide_height = 6
guide_length = 7
guide_width = suspension.wheel_gap - clearance
guide_side_angle = 30
guide_clearance_radius = 10
negative_bend_angle = 10
groove_height = 1

connector_width = 5
connector_length = 5 # Length between centers

base_thickness = vitamins.nail.diameter + segment_pin_clearance + 2 * wall
connector_thickness = vitamins.nail.diameter + connector_pin_clearance + 2 * wall
surface_offset = base_thickness / 2 # How high above the pivot is the surface that wheels contact
segment_width = width - 2 * (clearance + connector_width)
extra_foot_height = groove_height

assert suspension.wheel_width <= width + 2 * clearance
connector_length >= vitamins.nail.diameter + segment_pin_clearance + 2 * wall + clearance

def track_segment_generator(between_centers, width,
                            pivot_diameter, base_thickness,

                            connector_width, connector_thickness, connector_length,
                            extra_foot_height,
                            negative_bend_angle,

                            guide_height, guide_length, guide_width, guide_side_angle, guide_clearance_radius,
                            groove_width, groove_height,
                            clearance):

    segment = rectangle(width, between_centers + base_thickness).extruded(base_thickness + extra_foot_height).translated_z(-extra_foot_height / 2)

    # Round the sides to allow positive and negative bending
    c = math.cos(math.radians(negative_bend_angle))
    s = math.sin(math.radians(negative_bend_angle))
    cut = polygon2d([(-0.5 * s, 0.5 * c), (0.5, 0), (1, 0), (1, 1), (-1 - extra_foot_height, 1)]) - \
                    circle(d=1) + \
                    half_plane().translated_y(0.5).rotated(negative_bend_angle)
    cut = cut.scaled(base_thickness)
    cut += circle(d=pivot_diameter)
    cut = cut.translated_y(between_centers / 2)
    cut += cut.mirrored_y()
    segment -= cut.extruded(float("inf")).rotated_y(-90)

    # Make track guide
    guide = cylinder(r=guide_clearance_radius, h=guide_width).rotated_y(90)

    guide_side = half_space() \
                 .rotated_x(-guide_side_angle) \
                 .translated_y(-guide_width / 2) \
                 .translated_z(base_thickness / 2)

    guide = intersection([guide.translated_y(between_centers / 2),
                          guide.translated_y(-between_centers / 2),
                          rectangle(guide_width, guide_length).extruded(guide_height + base_thickness / 2, symmetrical=False),
                          guide_side.rotated_z(90),
                          guide_side.rotated_z(-90)])
    segment += guide

    connector_gap = polygon2d([(0, 0), (0, -base_thickness), (base_thickness, base_thickness)]) \
        .offset(connector_thickness / 2 + clearance / 2) \
        .extruded(connector_width) \
        .offset(clearance / 2) \
        .rotated_x(90) \
        .rotated_z(90) \
        .translated_y(between_centers / 2)
    segment -= connector_gap
    segment -= connector_gap.rotated_z(180)

    groove_angle = 30
    groove = box(2 * groove_height, groove_width, width + groove_width * math.sin(math.radians(groove_angle))) \
        .translated_z(width / 2) \
        .rotated_y(90) \
        .rotated_z(-30)

    groove_spacing = (between_centers + connector_length) / 3
    grooves = union([groove.translated_y(i * groove_spacing) for i in range(-1, 3)])
    grooves += grooves.rotated_y(180)
    segment -= grooves.translated_z(-base_thickness/2 - extra_foot_height)

    return segment.rotated_z(90)#.translated_z(base_thickness / 2 + extra_foot_height)


def track_connector_generator(between_centers, thickness, width, pivot_diameter, cone_depth, clearance):
    connector = rectangle(between_centers, 0).offset(thickness / 2)
    #connector -= circle(d=pivot_diameter).translated_x(between_centers / 2)
    #connector -= circle(d=pivot_diameter).translated_x(-between_centers / 2)
    connector = connector.extruded(width, symmetrical=False)

    tmp = pivot_diameter / 2 + cone_depth
    hole = polygon2d([(-pivot_diameter, width),
                      (tmp, width),
                      (tmp, width / 2),
                      (pivot_diameter / 2, width / 2 - cone_depth),
                      (pivot_diameter / 2, -width / 2 + cone_depth),
                      (tmp, -width / 2),
                      (tmp, -width),
                      (-pivot_diameter, -width)]) \
        .revolved() \
        .rotated_x(90) \
        .translated_z(width / 2)

    connector -= hole.translated_x(between_centers / 2)
    connector -= hole.translated_x(-between_centers / 2)
    return connector


track_segment = track_segment_generator(segment_length, segment_width,
                                        vitamins.nail.diameter + segment_pin_clearance,
                                        base_thickness,

                                        connector_width, connector_thickness, connector_length,
                                        groove_height, # Foot extra height
                                        negative_bend_angle,

                                        guide_height, guide_length, guide_width, guide_side_angle, guide_clearance_radius,
                                        2, # groove_width
                                        groove_height,
                                        clearance
                                        ).make_part("track_segment", ["3d_print"])
track_connector = track_connector_generator(connector_length, connector_thickness, connector_width,
                                            vitamins.nail.diameter + connector_pin_clearance,
                                            0.4, # Cone depth
                                            clearance
                                            ).make_part("track_connector", ["3d_print"])

def track_row(n, modelled_n=1):
    parts = []

    for i in range(modelled_n):
        dist = (i - n/2 + 0.5) * (segment_length + connector_length)

        conn = track_connector \
            .rotated_x(90) \
            .translated(dist + (segment_length + connector_length) / 2,
                        connector_width / 2,
                        0)

        parts.append(track_segment.translated_x(dist))
        parts.append(conn)
        parts.append(conn.translated_y((width - connector_width - clearance) / 2))
        parts.append(conn.translated_y(-(width - connector_width - clearance) / 2))

    hidden_n = n - modelled_n

    parts.extend(track_segment.hidden() for i in range(hidden_n))
    parts.extend(track_connector.hidden() for i in range(3 * hidden_n))
    parts.extend(vitamins.nail for i in range(4 * n))

    return codecad.assembly("track_assembly", parts)

track_assembly = track_row(120, 2)

if __name__ == "__main__":
    codecad.commandline_render(track_assembly, 0.05)
