import math

import scipy.optimize

import codecad
from codecad.shapes import *

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

def drive_sprocket_generator(n,
                             pitch_radius,
                             segment_length,
                             connector_length, connector_thickness,
                             plate_height, barrel_height, cone_height,
                             guide_gap_height):

    next_segment_end_angle = math.asin(connector_length / (2 * pitch_radius)) + \
                             2 * math.asin(segment_length / (2 * pitch_radius))
    connector_center_distance = math.sqrt(pitch_radius**2 - connector_length**2 / 4)
    s = math.sin(next_segment_end_angle) * pitch_radius
    c = math.cos(next_segment_end_angle) * pitch_radius

    # Side plate that drives the pins
    plate = circle(r=segment_length - connector_thickness / 2) \
        .translated(c, s)
    plate = plate + plate.mirrored_y() + circle(r=pitch_radius)
    plate -= rectangle(2 * connector_thickness, connector_length) \
        .translated_x(connector_thickness + connector_center_distance) \
        .offset(connector_thickness / 2)
    plate = plate.offset(-track.clearance / 2)
    plate &= circle(r=pitch_radius + connector_thickness)
    plate = plate.extruded(plate_height, symmetrical=False)

    # Inner barrell
    barrell = polygon2d([(-connector_thickness, s),
                         (c, s),
                         (connector_center_distance, track.connector_length / 2),
                         (connector_center_distance, -track.connector_length / 2),
                         (c, -s),
                         (-connector_thickness, -s)]) \
        .offset(-0.682 * connector_thickness) \
        .extruded(barrel_height, symmetrical=False)
        #TODO: Why does the offset need the fudge factor?

    print(cone_height, barrel_height)
    cone = tools.cone(height=cone_height - barrel_height,
                      upper_diameter=40 - guide_gap_height,
                      lower_diameter=40,
                      base_height=barrel_height)

    sprocket = plate + barrell + cone
    #sprocket = plate

    sprocket = codecad.shapes.unsafe.CircularRepetition(sprocket, n)

    return sprocket

drive_sprocket = drive_sprocket_generator(drive_sprocket_tooth_count,
                                          drive_sprocket_pitch_radius,
                                          track.segment_length,
                                          track.connector_length,
                                          track.connector_thickness,
                                          track.connector_width,
                                          (track.width - suspension.wheel_gap) / 2,
                                          (track.width - suspension.wheel_gap) / 2 + track.guide_height * math.sin(math.radians(track.guide_side_angle)),
                                          track.guide_height + track.clearance) \
    .make_part("drive_sprocket")

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

    codecad.commandline_render(drive_sprocket.rotated_z(-90), 0.1)
