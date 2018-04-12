import math

import scipy.optimize

import codecad
from codecad.shapes import *

import vitamins
import parameters
import track
import tools

tooth_count = 11

def pitch_radius_eq(r):
    alpha = 2 * math.asin(track.segment_length / (2 * r))
    beta = 2 * math.asin(track.connector_length / (2 * r))
    return alpha + beta - 2 * math.pi / tooth_count


pitch_radius = scipy.optimize.brentq(pitch_radius_eq,
                                     track.segment_length + track.connector_length,
                                     tooth_count * (track.segment_length + track.connector_length) / math.pi)
    # Diameter at which the track pivot centers ride
circumference = (track.segment_length + track.connector_length) * tooth_count
    # How much the track moves for one rotation of the drive sprocket
center_screw_wall_thickness = 3

def base_generator(n,
                   pitch_radius,
                   segment_length, segment_thickness,
                   connector_length, connector_thickness,
                   connector_width,
                   track_width,
                   guide_width, guide_height, guide_side_angle,
                   track_clearance):

    outer_radius = pitch_radius + connector_thickness * 0.75
    barrel_radius = math.sqrt(pitch_radius**2 - segment_length**2) #- o_ring_minor_diameter / 2
    mid_radius = barrel_radius - guide_height - track_clearance

    plate_height = connector_width
    barrel_height = (track_width - guide_width - track_clearance) / 2
    cone_height = barrel_height + math.tan(math.radians(guide_side_angle)) * (barrel_radius - mid_radius)
    total_height = track_width / 2

    sprocket = polygon2d([
        (outer_radius, 0),
        (outer_radius, plate_height / 2),
        (pitch_radius + connector_thickness / 2, plate_height),
        (barrel_radius, plate_height),
        (barrel_radius, barrel_height),
        (mid_radius, cone_height),
        (mid_radius, total_height),
        (0, total_height),
        (0, 0)])
    # sprocket -= circle(d=o_ring_minor_diameter).translated(barrel_radius, (plate_height + barrel_height) / 2)

    sprocket = sprocket \
        .revolved() \
        .rotated_x(90)

    # cutouts for the side plate
    next_segment_end_angle = math.asin(connector_length / (2 * pitch_radius)) + \
                             2 * math.asin(segment_length / (2 * pitch_radius))
    connector_center_distance = math.sqrt(pitch_radius**2 - connector_length**2 / 4)
    plate = circle(r=segment_length - connector_thickness / 2) \
        .translated(math.cos(next_segment_end_angle) * pitch_radius, math.sin(next_segment_end_angle) * pitch_radius)
    plate = plate + plate.mirrored_y() + circle(r=pitch_radius)
    plate -= rectangle(2 * connector_thickness, connector_length) \
        .translated_x(connector_thickness + connector_center_distance) \
        .offset(connector_thickness / 2)
    plate = plate.offset(-track.clearance / 2)
    plate = codecad.shapes.unsafe.CircularRepetition2D(plate, n)
    plate = plate.offset(-track_clearance / 2)
    #plate &= circle(r=pitch_radius + connector_thickness)
    #plate = plate.extruded(plate_height, symmetrical=False)

    sprocket &= plate.extruded(float("inf"))

    sprocket.screw_radius = 0.6 * mid_radius
    sprocket.screw_count = 3
    sprocket.thickness = total_height

    return sprocket

def inner_sprocket_generator(base,
                             spline, spline_tolerance,
                             bearing_shoulder_width, bearing_shoulder_height,
                             total_bearing_cutout_diameter,
                             half_screw_diameter,
                             remaining_half_screw_length,
                             half_screw_nut_s,
                             half_screw_nut_height,
                             hole_blinding_layer_height):
    half = base
    half -= spline.offset(spline_tolerance).extruded(float("inf"))

    r1 = spline.od / 2 + bearing_shoulder_width
    r3 = total_bearing_cutout_diameter / 2
    h = (r3 - r1 + bearing_shoulder_height) / 2
    r2 = r3 - h

    # Central cutout for the bearing housing
    half -= polygon2d([
        (r1, -10),
        (r1, bearing_shoulder_height),
        (r2, h),
        (r3, 0),
        (r3, -10)]) \
        .revolved() \
        .rotated_x(90)

    nut_plane = base.thickness - remaining_half_screw_length + half_screw_nut_height + half_screw_diameter / 6
    screw = cylinder(d=half_screw_diameter, h=float("inf"))
    screw += regular_polygon2d(n=6, across_flats=half_screw_nut_s) \
        .extruded(nut_plane,
                  symmetrical=False)
    if hole_blinding_layer_height:
        screw -= box(half_screw_diameter, half_screw_diameter, hole_blinding_layer_height) \
            .translated_z(nut_plane + hole_blinding_layer_height / 2)
    screw = screw.translated_x(base.screw_radius)
    half -= unsafe.CircularRepetition(screw, base.screw_count)

    return half

def outer_sprocket_generator(base,
                             center_screw_diameter,
                             center_screw_wall,
                             center_screw_head_diameter,
                             center_screw_head_height,

                             half_screw_diameter,
                             half_screw_head_diameter,
                             half_screw_head_height,

                             wall_thickness,
                             hole_blinding_layer_height):
    half = base

    # Central screw hole and its cone
    central_cutouts = polygon2d([
        (-10, -10),
        (-10, base.thickness + 10),
        (center_screw_diameter / 2, base.thickness + 10),
        (center_screw_diameter / 2, base.thickness - center_screw_wall),
        (center_screw_head_diameter / 2, base.thickness - center_screw_wall),
        (center_screw_head_diameter / 2, base.thickness - center_screw_wall - center_screw_head_height),
        (center_screw_head_diameter * 1.2 / 2,
            base.thickness - center_screw_wall - center_screw_head_height - center_screw_head_height * 0.1),
        (base.screw_radius - half_screw_head_diameter / 2 - wall_thickness, 0),
        (base.screw_radius - half_screw_head_diameter / 2 - wall_thickness, -10),
        ])

    # Decorative ring:
    central_cutouts += circle(d=1.5 * wall_thickness). \
        translated_x(base.screw_radius + half_screw_head_diameter / 2 + 3 * wall_thickness)

    if hole_blinding_layer_height:
        central_cutouts -= rectangle(center_screw_diameter, hole_blinding_layer_height) \
            .translated_y(base.thickness - center_screw_wall + hole_blinding_layer_height / 2)

    half -= central_cutouts \
        .revolved() \
        .rotated_x(90)


    screw = polygon2d([
        (-10, -10),
        (-10, base.thickness + 10),
        (half_screw_diameter / 2, base.thickness + 10),
        (half_screw_diameter / 2, half_screw_head_height),
        (half_screw_head_diameter / 2, half_screw_head_height),
        (half_screw_head_diameter / 2, -10)])

    if hole_blinding_layer_height:
        screw -= rectangle(half_screw_diameter, hole_blinding_layer_height) \
            .translated_y(half_screw_head_height + hole_blinding_layer_height / 2)

    screw = screw.revolved() \
        .rotated_x(90) \
        .translated_x(base.screw_radius)
    half -= unsafe.CircularRepetition(screw, base.screw_count)

    return half

base = base_generator(tooth_count,
                      pitch_radius,
                      track.segment_length,
                      track.base_thickness,
                      track.connector_length,
                      track.connector_thickness,
                      track.connector_width, track.width,
                      track.guide_width, track.guide_height, track.guide_side_angle,
                      track.clearance)
spline = tools.spline(vitamins.large_bearing.id)

inner_sprocket = inner_sprocket_generator(base,
                                          spline, 0.1,
                                          vitamins.large_bearing.shoulder_size, 3, 35,

                                          vitamins.small_screw.diameter,
                                          vitamins.small_screw.length + vitamins.small_screw.head_height - base.thickness,
                                          vitamins.small_screw.lock_nut.s,
                                          vitamins.small_screw.lock_nut.height,

                                          parameters.overhang_hole_blinding) \
    .make_part("inner_drive_sprocket", ["3d_print"])
outer_sprocket = outer_sprocket_generator(base,
                                          vitamins.small_screw.diameter,
                                          center_screw_wall_thickness,
                                          vitamins.long_screw.head_diameter,
                                          vitamins.long_screw.head_height,

                                          vitamins.small_screw.diameter,
                                          vitamins.small_screw.head_diameter,
                                          vitamins.small_screw.head_height,

                                          3 * parameters.extrusion_width,
                                          parameters.overhang_hole_blinding) \
    .make_part("outer_drive_sprocket", ["3d_print"])

drive_sprocket_assembly = codecad.assembly("drive_sprocket_assembly",
                                           [inner_sprocket.rotated_x(180).translated_z(track.width / 2),
                                            outer_sprocket.translated_z(-track.width / 2)] + \
                                           [vitamins.small_screw] * 3 + \
                                           [vitamins.small_screw.lock_nut] * 3)

if __name__ == "__main__":
    print("pitch radius", pitch_radius)

    codecad.commandline_render(drive_sprocket_assembly)
