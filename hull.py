import math

import codecad
from codecad.shapes import *
from codecad.util import Vector

import parameters
import tools
import tensioner
import suspension
import drive_sprocket
import track
import transmission

suspension_pivot_y = 30
tensioner_position = tensioner.pivot_position + Vector(0, suspension_pivot_y)
bogie_positions = [Vector(i * suspension.suspension_spacing + tensioner.to_suspension_pivot,
                          suspension_pivot_y)
                   for i in range(1)]
                   #for i in range(suspension.bogie_count // 2)]
drive_sprocket_position = Vector(bogie_positions[-1].x + drive_sprocket.to_suspension_pivot,
                                 suspension_pivot_y)

track_clearance = 5 # Distance between track and hull

def hull_generator(width,
                   track_center, # Distance between hull side and track centerline
                   tensioner_position, bogie_positions, drive_sprocket_position,
                   glacis_radius, front_angle, rear_angle,
                   base_thickness,
                   corner_frame_size,
                   skid_width, skid_height,

                   drive_sprocket_bearing,
                   drive_sprocket_bearing_shoulder_height,
                   drive_sprocket_bearing_housing_top_diameter,
                   drive_sprocket_bearing_track_offset, # Distance between track centerline and outer face of drive sprocket bearing
                   drive_sprocket_gear_clearance_radius,

                   mount_safety_distance):
    assert tensioner_position.x == 0

    sin_rear = math.sin(math.radians(rear_angle))
    cos_rear = math.cos(math.radians(rear_angle))
    tan_rear = math.tan(math.radians(rear_angle))
    sin_front = math.sin(math.radians(front_angle))
    cos_front = math.cos(math.radians(front_angle))
    tan_front = math.tan(math.radians(front_angle))

    drive_sprocket_cone_height = track_center - drive_sprocket_bearing_track_offset + drive_sprocket_bearing_shoulder_height
    drive_sprocket_bearing_housing_lower_diameter = drive_sprocket_bearing_housing_top_diameter + 2 * drive_sprocket_cone_height
    drive_sprocket_clearance_radius = max(drive_sprocket_bearing_housing_lower_diameter / 2, drive_sprocket_gear_clearance_radius)

    mount_points = [tensioner_position] + bogie_positions + [drive_sprocket_position]
    height = max(point.y for point in mount_points) + mount_safety_distance
    height = max(height, drive_sprocket_position.y + drive_sprocket_clearance_radius)

    half_hull = box(mount_points[-1].x, width, 2 * height) \
        .translated_z(height) \

    def extension(mount_safety_distance, y, angle):
        return (mount_safety_distance - y * math.sin(math.radians(angle))) / math.cos(math.radians(angle))

    front_ext = extension(mount_safety_distance,
                          tensioner_position.y,
                          front_angle)
    front_ext_top = front_ext + tan_front * height
    back_ext = extension(drive_sprocket_clearance_radius,
                         drive_sprocket_position.y,
                         rear_angle)
    back_ext_top = back_ext + tan_rear * height

    radius_tmp = glacis_radius / math.tan(math.radians(front_angle / 2 + 45))

    side_profile = polygon2d([
        (tensioner_position.x - front_ext + radius_tmp, 0),
        (tensioner_position.x - front_ext - sin_front * radius_tmp,
         cos_front * radius_tmp),
        (tensioner_position.x - front_ext_top, height),
        (tensioner_position.x - front_ext_top, 2 * height),
        (drive_sprocket_position.x + back_ext_top, 2 * height),
        (drive_sprocket_position.x + back_ext_top, height),
        (drive_sprocket_position.x + back_ext, 0),
        ])
    side_profile += circle(r=glacis_radius).translated(tensioner_position.x - front_ext + radius_tmp, glacis_radius)

    half_hull = side_profile \
        .extruded(width) \
        .rotated_x(90)

    # Shell the hull and cut off the upper half
    half_hull = half_hull.offset(-base_thickness / 2).shell(base_thickness) & half_space().rotated_x(-90).translated_z(height)
    side_profile = side_profile & half_plane().rotated(180).translated_y(height)

    # The corner part of side panel frame
    side_frame = side_profile \
        .offset(-corner_frame_size / 2 - base_thickness / 2) \
        .shell(corner_frame_size + base_thickness)

    # Members of the frame going across the hull
    #frame_across = polygon2d([
    #    (, height - corner_frame_size)

    # Add drive sprocket ribs to side frame
    side_frame += (
        rectangle(0.4 * drive_sprocket_bearing_housing_lower_diameter, drive_sprocket_bearing_housing_lower_diameter) \
            .translated_y(drive_sprocket_bearing_housing_lower_diameter / 2) \
            .rotated(-90 - rear_angle)
        + rectangle(corner_frame_size, 2 * drive_sprocket_bearing_housing_lower_diameter)) \
        .translated(drive_sprocket_position.x, drive_sprocket_position.y) \
        & side_profile

    # Add side panel frame to hull
    half_hull += side_frame \
        .extruded(base_thickness + corner_frame_size, symmetrical=False) \
        .rotated_x(90) \
        .translated_y(width / 2)

    # Add skids to the hull
    skid_base = side_profile \
        .offset((skid_height - base_thickness) / 2) \
        .shell(base_thickness + skid_height)
    skid_base &= polygon2d([
        (tensioner_position.x - front_ext_top, height),
        #(drive_sprocket_position.x + back_ext, skid_height + base_thickness),
        (drive_sprocket_position.x + back_ext, 0),
        (tensioner_position.x - front_ext_top, -(drive_sprocket_position.x + back_ext) / tan_rear)
        ])
    skid = skid_base \
        .extruded(skid_width) \
        .rotated_x(90)
    half_hull += skid.translated_y(width / 2 - skid_width / 2)
    half_hull += skid.translated_y((width - skid_width) / 6)

    # Drive sprocket bearing housing cone
    half_hull += tools.cone(height=drive_sprocket_cone_height,
                            upper_diameter=drive_sprocket_bearing_housing_top_diameter,
                            lower_diameter=drive_sprocket_bearing_housing_lower_diameter,
                            base_height=base_thickness + corner_frame_size) \
        .rotated_x(-90) \
        .translated(drive_sprocket_position.x, width / 2 - base_thickness - corner_frame_size, drive_sprocket_position.y)

    # Drive sprocket bearing housing cutout
    half_hull -= polygon2d([
        (-5, 2 * drive_sprocket_cone_height),
        (drive_sprocket_bearing.od / 2 - drive_sprocket_bearing.shoulder_size, 2 * drive_sprocket_cone_height),
        (drive_sprocket_bearing.od / 2 - drive_sprocket_bearing.shoulder_size, drive_sprocket_cone_height + base_thickness - drive_sprocket_bearing_shoulder_height),
        (drive_sprocket_bearing.od / 2,  drive_sprocket_cone_height + base_thickness - drive_sprocket_bearing_shoulder_height),
        (drive_sprocket_bearing.od / 2,  drive_sprocket_cone_height + base_thickness - drive_sprocket_bearing_shoulder_height - drive_sprocket_bearing.thickness),
        #(drive_sprocket_bearing.od / 2 + drive_sprocket_cone_height + base_thickness - drive_sprocket_bearing_shoulder_height - drive_sprocket_bearing.thickness, 0),
        (drive_sprocket_bearing.od / 2 + drive_sprocket_cone_height + base_thickness - drive_sprocket_bearing_shoulder_height - drive_sprocket_bearing.thickness + drive_sprocket_cone_height, -drive_sprocket_cone_height),
        (-5, -drive_sprocket_cone_height),
        ]) \
        .revolved() \
        .translated(drive_sprocket_position.x, width / 2 - base_thickness, drive_sprocket_position.y)


    # Placeholder mount point cylinders
    half_hull += union([cylinder(r=mount_safety_distance, h=10, symmetrical=False)
                            .rotated_x(-90)
                            .translated(mp.x, width / 2, mp.y)
                        for mp in mount_points[:-1]])

    hull = half_hull.symmetrical_y()

    return hull.rotated_y(90 - rear_angle)


hull = hull_generator(180, # Width
                      track.width / 2 + track_clearance,
                      tensioner_position, bogie_positions, drive_sprocket_position,
                      120, # Glacis radius
                      60, 15, # Front and rear angle
                      4 * parameters.extrusion_width, # Base thickness
                      5, # Frame size
                      10, 2, # Skid width, skid height

                      drive_sprocket.bearing,
                      drive_sprocket.bearing_shoulder_height,
                      drive_sprocket.bearing_housing_top_diameter,
                      track.width / 2,
                      transmission.final_drive_gear_radius + transmission.gear_tip_clearance,


                      15) \
    .make_part("hull", ["3d_print"])# \
    #.rotated_y(-90 + 15)

if __name__ == "__main__":
    def p(name, f=lambda x: x):
        print(name, f(globals()[name]))

    p("tensioner_position")
    p("bogie_positions")
    p("drive_sprocket_position")

    codecad.commandline_render((hull.shape() & half_space()))

import sys
sys.exit()

thin_wall = 5 * parameters.extrusion_width

# Depth of the shoulder screw shaft in the side panel, including the spacing knob
pivot_screw_diameter2_depth = parameters.shoulder_screw_length - parameters.shoulder_screw_screw_length \
    - suspension.pivot_guide_length - suspension.pivot_flat_clearance

side_thickness = pivot_screw_diameter2_depth - suspension.arm_clearance + thin_wall + \
    max(parameters.large_screw_nut_height, parameters.shoulder_screw_nut_height) - suspension.pivot_flat_clearance
side_pivot_height = suspension.arm_clearance - suspension.spring_anchor_point.z

def side_generator(width, height, thickness,
                   arm_clearance,
                   suspension_pivot_z, suspension_pivot_diameter1, suspension_pivot_diameter2,
                   suspension_pivot_diameter2_depth,
                   suspension_pivot_height, suspension_pivot_nut_s,

                   spring_anchor_point, spring_up_point, spring_down_point, spring_anchor_diameter,
                   spring_anchor_nut_s,

                   thin_wall,
                   hole_blinding_layer_height):

    side = box(width, height, thickness) \
        .translated(spring_anchor_point.x / 2, height / 2 - suspension_pivot_z, thickness / 2)

    side += tools.cone(height=suspension_pivot_height,
                       upper_diameter=suspension_pivot_diameter2 + 2 * thin_wall,
                       lower_diameter=suspension_pivot_diameter2 + 2 * thin_wall + 2 * suspension_pivot_height,
                       base_height=thickness)
    side += tools.cone(height=arm_clearance,
                       upper_diameter=spring_anchor_diameter + 2 * thin_wall,
                       lower_diameter=spring_anchor_diameter + 2 * thin_wall + 2 * arm_clearance,
                       base_height=thickness) \
        .translated(spring_anchor_point.x, spring_anchor_point.y, 0)

    spring_up_vector = spring_up_point - spring_anchor_point
    spring_up_angle = math.degrees(math.atan2(spring_up_vector.y, spring_up_vector.x))
    spring_down_vector = spring_down_point - spring_anchor_point
    spring_down_angle = math.degrees(math.atan2(spring_down_vector.y, spring_down_vector.x))
    side -= suspension.spring_cutout_generator(spring_angle=spring_up_angle - spring_down_angle,
                                               r0=4,
                                               r1=suspension.spring_length,
                                               chamfer0=0,
                                               chamfer1=suspension.spring_diameter / 2) \
        .rotated_z(spring_down_angle) \
        .translated(spring_anchor_point.x, spring_anchor_point.y, thickness + arm_clearance + suspension.spring_top_mount_thickness / 2)

    # Nut depth is measured from the back side
    nut_depth = thickness + arm_clearance - suspension_pivot_diameter2_depth - thin_wall

    side -= cylinder(d=suspension_pivot_diameter1, h=float("inf"))
    side -= cylinder(d=suspension_pivot_diameter2, h=2 * suspension_pivot_diameter2_depth) \
        .translated_z(thickness + arm_clearance)
    side -= regular_polygon2d(n=6, d=suspension_pivot_nut_s * 2 / math.sqrt(3)) \
        .extruded(2 * nut_depth)

    side -= cylinder(d=spring_anchor_diameter, h=float("inf")) \
        .translated(spring_anchor_point.x, spring_anchor_point.y, 0)
    side -= regular_polygon2d(n=6, d=spring_anchor_nut_s * 2 / math.sqrt(3)) \
        .extruded(2 * nut_depth) \
        .translated(spring_anchor_point.x, spring_anchor_point.y, 0)

    if hole_blinding_layer_height:
        side += cylinder(d=suspension_pivot_nut_s * 2, h=hole_blinding_layer_height, symmetrical=False) \
            .translated_z(nut_depth)
        side += cylinder(d=spring_anchor_nut_s * 2, h=hole_blinding_layer_height, symmetrical=False) \
            .translated(spring_anchor_point.x, spring_anchor_point.y, nut_depth)

    side = side.translated_y(suspension_pivot_z)

    return side

test_side = side_generator(80, 35, side_thickness,
                           suspension.arm_clearance,

                           suspension_pivot_z,
                           parameters.shoulder_screw_diameter, parameters.shoulder_screw_diameter2,
                           pivot_screw_diameter2_depth,
                           side_pivot_height, parameters.shoulder_screw_nut_s,

                           suspension.spring_anchor_point, suspension.spring_up_point, suspension.spring_down_point,
                           suspension.spring_top_mount_id, parameters.large_screw_nut_s,

                           thin_wall,
                           parameters.layer_height
                           ).make_part("test_side")

if __name__ == "__main__":
    codecad.commandline_render(test_side.rotated_x(90))
