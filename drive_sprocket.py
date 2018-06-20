import math

import scipy.optimize

import codecad
from codecad.shapes import *

import vitamins
import parameters
import track
import tools

from parameters import wheel_clearance

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
to_suspension_pivot = 145 # TODO: Calculate this
bearing = vitamins.large_bearing
bearing_shoulder_height = parameters.extrusion_width * 6
bearing_housing_top_diameter = bearing.od + 2 * 4 * parameters.extrusion_width

def base_generator(n,
                   pitch_radius,
                   segment_length, segment_thickness,
                   connector_length, connector_thickness,
                   connector_width,
                   track_width,
                   guide_width, guide_height, guide_side_angle,
                   track_clearance):

    outer_radius = pitch_radius + connector_thickness * 0.75
    barrel_radius = math.sqrt(pitch_radius**2 - (segment_length / 2)**2) - track.base_thickness / 2
    mid_radius = barrel_radius - guide_height - track_clearance

    plate_height = connector_width
    barrel_height = (track_width - guide_width - track_clearance) / 2
    cone_height = barrel_height + math.tan(math.radians(guide_side_angle)) * (barrel_radius - mid_radius)
    total_height = track_width - cone_height

    sprocket = polygon2d([
        (outer_radius, 0),
        (outer_radius, plate_height / 2),
        (pitch_radius + connector_thickness / 4, plate_height),
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
    plate = unsafe.CircularRepetition2D(plate, n)
    plate = plate.offset(-track_clearance / 2)
    #plate &= circle(r=pitch_radius + connector_thickness)
    #plate = plate.extruded(plate_height, symmetrical=False)

    sprocket &= plate.extruded(float("inf"))

    sprocket.total_height = total_height
    sprocket.cone_height = cone_height
    sprocket.mid_radius = mid_radius

    return sprocket

def inner_sprocket_generator(base,
                             spline, spline_tolerance,
                             crown_tolerance,
                             bearing_shoulder_width, bearing_shoulder_height,
                             total_bearing_cutout_diameter):
    half = base

    r1 = spline.od / 2 + bearing_shoulder_width
    r3 = total_bearing_cutout_diameter / 2
    r2 = r3 - bearing_shoulder_height

    # Central cutout for the bearing housing
    half -= polygon2d([
        (r1, -10),
        (r1, bearing_shoulder_height),
        (r2, bearing_shoulder_height),
        (r3, 0),
        (r3, -10)]) \
        .revolved() \
        .rotated_x(90)

    half += unsafe.CircularRepetition2D(rectangle(r3,
                                                  parameters.overhang_spokes_width).translated_x(r3 / 2),
                                        5) \
        .extruded(2 * parameters.overhang_spokes_height) \
        .translated_z(bearing_shoulder_height)

    half -= spline.offset(spline_tolerance).extruded(float("inf"))

    half -= tools.crown_cutout(outer_diameter=2*base.mid_radius,
                               inner_diameter=spline.od / 2 + base.mid_radius,
                               tolerance=crown_tolerance,
                               height=base.total_height - base.cone_height,
                               inverse=True) \
        .translated_z(base.total_height)

    return half

def outer_sprocket_generator(base,
                             crown_tolerance,
                             center_screw,
                             center_screw_wall,

                             wall_thickness):
    half = base

    screw_head_height = (base.total_height + base.cone_height) / 2 - center_screw_wall

    # Central screw hole and its cone
    half -= polygon2d([
        (-10, -10),
        (-10, base.total_height + 10),
        (center_screw.diameter / 2, base.total_height + 10),
        (center_screw.diameter / 2, screw_head_height),
        (center_screw.head_diameter / 2, screw_head_height),
        (center_screw.head_diameter / 2, screw_head_height - center_screw.head_height),
        (center_screw.head_diameter / 2 + screw_head_height - center_screw.head_height, 0)]) \
        .revolved() \
        .rotated_x(90)

    if parameters.overhang_hole_blinding:
        half += cylinder(r=center_screw.diameter, h=parameters.overhang_hole_blinding, symmetrical=False) \
            .translated_z(screw_head_height)

    half -= tools.crown_cutout(outer_diameter=2*base.mid_radius,
                               inner_diameter=spline.od / 2 + base.mid_radius,
                               tolerance=crown_tolerance,
                               height=base.total_height - base.cone_height,
                               inverse=False) \
        .translated_z(base.total_height)

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
spline = tools.spline(bearing.id)

inner_sprocket = inner_sprocket_generator(base,
                                          spline, 0.05,
                                          0.1, # Crown tolerance
                                          bearing.shoulder_size,
                                          bearing_shoulder_height + wheel_clearance,
                                          bearing_housing_top_diameter + 2 * wheel_clearance) \
    .make_part("inner_drive_sprocket", ["3d_print"])
outer_sprocket = outer_sprocket_generator(base,
                                          0.1, # Crown tolerance
                                          vitamins.m3x35_screw,
                                          center_screw_wall_thickness,

                                          3 * parameters.extrusion_width) \
    .make_part("outer_drive_sprocket", ["3d_print"])

drive_sprocket_assembly = codecad.assembly("drive_sprocket_assembly",
                                           [inner_sprocket.rotated_x(90).translated_y(track.width / 2),
                                            outer_sprocket.rotated_x(-90).translated_y(-track.width / 2)])

if __name__ == "__main__":
    print("pitch radius", pitch_radius)

    codecad.commandline_render(drive_sprocket_assembly)
