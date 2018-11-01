import math

import codecad
from codecad.shapes import *
from codecad.util import Vector
import parametric

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

def top_back_cross_frame_member(angle, size, wall_thickness):
    """ Return a 2D shape with the profile of the frame member going across the tank on the top back side """

    # Initial guesses for point positions:
    pl = parametric.objects.Polyline([(0, 0),
                                      (-size, 0),
                                      (-size, -size),
                                      (0, -size)])

    # Create the solver and set constraints:
    s = parametric.solver.Solver()
    s.auto_solve = False
    s.add_constraint(parametric.constraints.VariableFixed(pl[0].x))
    s.add_constraint(parametric.constraints.VariableFixed(pl[0].y))
    s.add_constraint(parametric.constraints.AbsoluteAngle(pl.line_segments[3], 90 + -angle))
    s.add_constraint(parametric.constraints.Horizontal(pl[0], pl[1]))
    s.add_constraint(parametric.constraints.Perpendicular(pl.line_segments[0], pl.line_segments[1]))
    s.add_constraint(parametric.constraints.Perpendicular(pl.line_segments[2], pl.line_segments[3]))
    s.add_constraint(parametric.constraints.Length(pl.line_segments[1], size))
    s.add_constraint(parametric.constraints.Length(pl.line_segments[2], size + wall_thickness))
    s.solve()

    # Polygon2d is usable as a vector directly
    return polygon2d(pl)


def bottom_front_cross_frame_member(max_angle, front_angle, radius, size, wall_thickness):
    """ Return a 2D shape with the profile of the frame member going across the tank on the bottom front side """
    pl = parametric.objects.Polyline([(0, 0),
                                      (0, -2 * radius),
                                      (-radius, -2*radius)])
    return None  # TODO!


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

                   mount_safety_distance,
                   max_overhang_angle):
    assert tensioner_position.x == 0

    assert rear_angle < max_overhang_angle, "Floor of the hull would not be printable because the rear angle causes too much overhang"
    max_angle = max_overhang_angle + rear_angle # Maximal angle measured from hull floor that can be printed with acceptable overhangs (90° is up, 0° is forward)
    assert 90 - front_angle <= max_angle, "Front of the hull would not be printable because the front angle causes too much overhang"


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
    glacis_radius_center_x = tensioner_position.x - front_ext + radius_tmp
    glacis_radius_center_y = glacis_radius
    side_profile += circle(r=glacis_radius).translated(glacis_radius_center_x, glacis_radius_center_y)

    half_hull = side_profile \
        .extruded(width) \
        .rotated_x(90)

    # Shell the hull and cut off the upper half
    half_hull = half_hull.offset(-base_thickness / 2).shell(base_thickness) & half_space().rotated_x(-90).translated_z(height)
    side_profile_extended = side_profile & half_plane().rotated(180).translated_y(height + base_thickness) # To compensate for missing wall thickness on the top side
    side_profile = side_profile & half_plane().rotated(180).translated_y(height)

    # The corner part of side panel frame
    side_frame = side_profile_extended \
        .offset(-corner_frame_size / 2 - base_thickness / 2) \
        .shell(corner_frame_size + base_thickness)
    side_frame = side_frame & side_profile

    # Members of the frame going across the hull
    cross_frame = top_back_cross_frame_member(rear_angle, corner_frame_size, base_thickness) \
        .translated(drive_sprocket_position.x + back_ext_top, height)
    cross_frame += rectangle(2 * (corner_frame_size + base_thickness), 2 * (corner_frame_size + base_thickness)) \
        .translated(drive_sprocket_position.x + back_ext - base_thickness * (sin_rear - 1), 0) \
        & side_frame
    tmp_polygon = top_back_cross_frame_member(front_angle, corner_frame_size, base_thickness) # Reusing the top back frame polygon, because the structure is the same, even though the final shape is not
    cross_frame += half_plane() \
        .rotated(180 - max_angle) \
        .translated(tensioner_position.x - front_ext_top - tmp_polygon.points[2, 0], height - corner_frame_size)
    cross_frame += polygon2d([(0, 0),
                              (math.cos(math.radians(-max_angle)), math.sin(math.radians(-max_angle))),
                              (math.cos(math.radians(90 + front_angle + max_angle)), math.sin(math.radians(90 + front_angle + max_angle))),
                              ]) \
        .scaled(2 * glacis_radius) \
        .translated(glacis_radius_center_x - 10, glacis_radius_center_y - 60)
    cross_frame &= side_frame # Avoid spilling the frame outside and inside of the tank

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

    # Add cross frame to hull
    half_hull += cross_frame.extruded(width).rotated_x(90)


    # Add skids to the hull
    skid_base = side_profile \
        .offset((skid_height - base_thickness) / 2) \
        .shell(base_thickness + skid_height)
    skid_base &= polygon2d([
        (tensioner_position.x - front_ext_top, height),
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


hull_rear_angle = 15
hull = hull_generator(180, # Width
                      track.width / 2 + track_clearance,
                      tensioner_position, bogie_positions, drive_sprocket_position,
                      80, # Glacis radius
                      45, hull_rear_angle, # Front and rear angle
                      4 * parameters.extrusion_width, # Base thickness
                      5, # Frame size
                      10, 2, # Skid width, skid height

                      drive_sprocket.bearing,
                      drive_sprocket.bearing_shoulder_height,
                      drive_sprocket.bearing_housing_top_diameter,
                      track.width / 2,
                      transmission.final_drive_gear_radius + transmission.gear_tip_clearance,

                      15,
                      parameters.max_overhang_angle) \
    .make_part("hull", ["3d_print"]) \
    .rotated_y(-90 + hull_rear_angle)

if __name__ == "__main__":
    def p(name, f=lambda x: x):
        print(name, f(globals()[name]))

    p("tensioner_position")
    p("bogie_positions")
    p("drive_sprocket_position")

    codecad.commandline_render((hull.shape() & half_space()))

#thin_wall = 5 * parameters.extrusion_width
#
## Depth of the shoulder screw shaft in the side panel, including the spacing knob
#pivot_screw_diameter2_depth = parameters.shoulder_screw_length - parameters.shoulder_screw_screw_length \
#    - suspension.pivot_guide_length - suspension.pivot_flat_clearance
#
#side_thickness = pivot_screw_diameter2_depth - suspension.arm_clearance + thin_wall + \
#    max(parameters.large_screw_nut_height, parameters.shoulder_screw_nut_height) - suspension.pivot_flat_clearance
#side_pivot_height = suspension.arm_clearance - suspension.spring_anchor_point.z
#
#def side_generator(width, height, thickness,
#                   arm_clearance,
#                   suspension_pivot_z, suspension_pivot_diameter1, suspension_pivot_diameter2,
#                   suspension_pivot_diameter2_depth,
#                   suspension_pivot_height, suspension_pivot_nut_s,
#
#                   spring_anchor_point, spring_up_point, spring_down_point, spring_anchor_diameter,
#                   spring_anchor_nut_s,
#
#                   thin_wall,
#                   hole_blinding_layer_height):
#
#    side = box(width, height, thickness) \
#        .translated(spring_anchor_point.x / 2, height / 2 - suspension_pivot_z, thickness / 2)
#
#    side += tools.cone(height=suspension_pivot_height,
#                       upper_diameter=suspension_pivot_diameter2 + 2 * thin_wall,
#                       lower_diameter=suspension_pivot_diameter2 + 2 * thin_wall + 2 * suspension_pivot_height,
#                       base_height=thickness)
#    side += tools.cone(height=arm_clearance,
#                       upper_diameter=spring_anchor_diameter + 2 * thin_wall,
#                       lower_diameter=spring_anchor_diameter + 2 * thin_wall + 2 * arm_clearance,
#                       base_height=thickness) \
#        .translated(spring_anchor_point.x, spring_anchor_point.y, 0)
#
#    spring_up_vector = spring_up_point - spring_anchor_point
#    spring_up_angle = math.degrees(math.atan2(spring_up_vector.y, spring_up_vector.x))
#    spring_down_vector = spring_down_point - spring_anchor_point
#    spring_down_angle = math.degrees(math.atan2(spring_down_vector.y, spring_down_vector.x))
#    side -= suspension.spring_cutout_generator(spring_angle=spring_up_angle - spring_down_angle,
#                                               r0=4,
#                                               r1=suspension.spring_length,
#                                               chamfer0=0,
#                                               chamfer1=suspension.spring_diameter / 2) \
#        .rotated_z(spring_down_angle) \
#        .translated(spring_anchor_point.x, spring_anchor_point.y, thickness + arm_clearance + suspension.spring_top_mount_thickness / 2)
#
#    # Nut depth is measured from the back side
#    nut_depth = thickness + arm_clearance - suspension_pivot_diameter2_depth - thin_wall
#
#    side -= cylinder(d=suspension_pivot_diameter1, h=float("inf"))
#    side -= cylinder(d=suspension_pivot_diameter2, h=2 * suspension_pivot_diameter2_depth) \
#        .translated_z(thickness + arm_clearance)
#    side -= regular_polygon2d(n=6, d=suspension_pivot_nut_s * 2 / math.sqrt(3)) \
#        .extruded(2 * nut_depth)
#
#    side -= cylinder(d=spring_anchor_diameter, h=float("inf")) \
#        .translated(spring_anchor_point.x, spring_anchor_point.y, 0)
#    side -= regular_polygon2d(n=6, d=spring_anchor_nut_s * 2 / math.sqrt(3)) \
#        .extruded(2 * nut_depth) \
#        .translated(spring_anchor_point.x, spring_anchor_point.y, 0)
#
#    if hole_blinding_layer_height:
#        side += cylinder(d=suspension_pivot_nut_s * 2, h=hole_blinding_layer_height, symmetrical=False) \
#            .translated_z(nut_depth)
#        side += cylinder(d=spring_anchor_nut_s * 2, h=hole_blinding_layer_height, symmetrical=False) \
#            .translated(spring_anchor_point.x, spring_anchor_point.y, nut_depth)
#
#    side = side.translated_y(suspension_pivot_z)
#
#    return side
#
#test_side = side_generator(80, 35, side_thickness,
#                           suspension.arm_clearance,
#
#                           suspension_pivot_z,
#                           parameters.shoulder_screw_diameter, parameters.shoulder_screw_diameter2,
#                           pivot_screw_diameter2_depth,
#                           side_pivot_height, parameters.shoulder_screw_nut_s,
#
#                           suspension.spring_anchor_point, suspension.spring_up_point, suspension.spring_down_point,
#                           suspension.spring_top_mount_id, parameters.large_screw_nut_s,
#
#                           thin_wall,
#                           parameters.layer_height
#                           ).make_part("test_side")
#
#if __name__ == "__main__":
#    codecad.commandline_render(test_side.rotated_x(90))
