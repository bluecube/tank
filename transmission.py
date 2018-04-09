import math

import scipy.optimize

import codecad
from codecad.shapes import *

import vitamins
import parameters
import suspension
import track
import tools

drive_sprocket_tooth_count = 11
transmission_steps = [(17, 39), (17, 39)]
    # Tooth counts of the transmission steps

motor_max_rpm = 930 * 11.6 # rpm, motor Kv * motor voltage

gear_tip_clearance = 1 # mm, distance between gear OD and neighboring geometry
module = 1.5

def pitch_radius_eq(r):
    alpha = 2 * math.asin(track.segment_length / (2 * r))
    beta = 2 * math.asin(track.connector_length / (2 * r))
    return alpha + beta - 2 * math.pi / drive_sprocket_tooth_count


drive_sprocket_pitch_radius = scipy.optimize.brentq(pitch_radius_eq,
                                                    track.segment_length + track.connector_length,
                                                    drive_sprocket_tooth_count * (track.segment_length + track.connector_length) / math.pi)
    # Diameter at which the track pivot centers ride
drive_sprocket_circumference = (track.segment_length + track.connector_length) * drive_sprocket_tooth_count
    # How much the track moves for one rotation of the drive sprocket
drive_sprocket_center_screw_wall_thickness = 3

def drive_sprocket_base_generator(n,
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

def inner_drive_sprocket_generator(base,
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

def outer_drive_sprocket_generator(base,
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


_drive_sprocket_base = drive_sprocket_base_generator(drive_sprocket_tooth_count,
                                                     drive_sprocket_pitch_radius,
                                                     track.segment_length,
                                                     track.base_thickness,
                                                     track.connector_length,
                                                     track.connector_thickness,
                                                     track.connector_width, track.width,
                                                     track.guide_width, track.guide_height, track.guide_side_angle,
                                                     track.clearance)
_drive_sprocket_spline = tools.spline(vitamins.large_bearing.id)
_gear_shaft2_spline = tools.spline(20)

inner_drive_sprocket = inner_drive_sprocket_generator(_drive_sprocket_base,
                                                      _drive_sprocket_spline, 0.1,
                                                      vitamins.large_bearing.shoulder_size, 3, 35,

                                                      vitamins.small_screw.diameter,
                                                      vitamins.small_screw.length + vitamins.small_screw.head_height - _drive_sprocket_base.thickness,
                                                      vitamins.small_screw.lock_nut.s,
                                                      vitamins.small_screw.lock_nut.height,

                                                      parameters.overhang_hole_blinding) \
    .make_part("inner_drive_sprocket", ["3d_print"])
outer_drive_sprocket = outer_drive_sprocket_generator(_drive_sprocket_base,
                                                      vitamins.small_screw.diameter,
                                                      drive_sprocket_center_screw_wall_thickness,
                                                      vitamins.long_screw.head_diameter,
                                                      vitamins.long_screw.head_height,

                                                      vitamins.small_screw.diameter,
                                                      vitamins.small_screw.head_diameter,
                                                      vitamins.small_screw.head_height,

                                                      3 * parameters.extrusion_width,
                                                      parameters.overhang_hole_blinding) \
    .make_part("outer_drive_sprocket", ["3d_print"])
shaft2 = shaft2_generator(_drive_sprocket_spline, _gear_shaft2_spline, 0.1,
                         vitamins.large_bearing,
                         _drive_sprocket_base.thickness,
                         vitamins.long_screw.diameter,
                         vitamins.long_screw.length - drive_sprocket_center_screw_wall_thickness,
                         vitamins.long_screw.lock_nut.s,
                         vitamins.long_screw.lock_nut.height,
                         parameters.overhang_hole_blinding) \
    .make_part("transmission_shaft2", ["3d_print"])

drive_sprocket_assembly = codecad.assembly("drive_sprocket_assembly",
                                           [inner_drive_sprocket.rotated_x(180).translated_z(track.width / 2 + 0.5),
                                            outer_drive_sprocket.translated_z(-track.width / 2 - 0.5)] + \
                                           [vitamins.small_screw] * 3 + \
                                           [vitamins.small_screw.lock_nut] * 3)

shaft2_assembly = codecad.assembly("shaft2_assembly",
                                   [drive_sprocket_assembly,
                                    shaft2.rotated_x(180).translated_z(shaft2.part.data.length + 20),
                                    vitamins.long_screw,
                                    vitamins.long_screw.lock_nut])

if __name__ == "__main__":
    print("drive sprocket working radius", drive_sprocket_pitch_radius)

    ratio = 1
    for t1, t2 in transmission_steps:
        assert math.gcd(t1, t2) == 1
        ratio *= t2 / t1

    speed = (motor_max_rpm / 60) * drive_sprocket_circumference / ratio
    print("transmission ratios",
          ", ".join("{}:{}".format(*teeth) for teeth in transmission_steps),
          "({:.1f}:1) -> {:.1f}m/s, {:.1f}km/h".format(ratio, speed/1000, 3.6 * speed / 1000))

    codecad.commandline_render((shaft2_assembly.shape().rotated_y(-90) & half_space()).rotated_z(-30).rotated_x(15), 0.1)
