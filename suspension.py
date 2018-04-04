import math

import scipy.optimize

import codecad
from codecad.shapes import *

import tools
import vitamins
import parameters

bogie_count = 6 # Count of bogies on both sides of the vehicle

spring_length = 62 # Center to center, relaxed
spring_travel = 11
spring_diameter = 17.1
spring_top_mount_id = 5
spring_top_mount_od = 8.5
spring_top_mount_thickness = 3.8
spring_bottom_mount_id = 3
spring_bottom_mount_od = 9
spring_bottom_mount_thickness = 6.5
spring_preload_force = 0.95 # [kg]
spring_full_compression_force = 4.5 # [kg]

suspension_spacing = 120

arm_clearance = 1
wheel_clearance = 2
pivot_flat_clearance = 0.5
pivot_round_clearance = 0.15

wheel_diameter = 34
wheel_width = vitamins.small_screw.length + vitamins.small_screw.head_height # Total width of the wheel pair

arm_width = 8
arm_thickness = vitamins.shoulder_screw.diameter2 + 10 * parameters.extrusion_width
arm_knee_height = 14
arm_knee_angle = 15

bogie_wheel_spacing = 55 # [mm] distance between wheels of one bogie
bogie_width = arm_width
bogie_pivot_z = 7
bogie_arm_cutout_angle = 75 # Degrees

suspension_min_travel = 30 # [mm]
suspension_sag = 0.3 # Ratio of travel from neutral position down

pivot_guide_length = 10 # How long part of the pivot screw shaft is kept inside the arm
pivot_screw_head_countersink = 4.5

wheel_gap = bogie_width + 2 * wheel_clearance
half_wheel_width = (wheel_width - wheel_gap) / 2
bogie_swing_angle = math.radians(bogie_arm_cutout_angle - arm_knee_angle) / 2

def point_to_line_distance(p, l1, l2):
    """ Calculate signed perpendicular distance from p to l1-l2 """
    u = (l2 - l1).normalized()
    v = codecad.util.Vector(u.y, -u.x)

    return (p - l1).dot(v)

def get_spring_point(spring_arm_length, angle):
    """ Return coordinates of the spring attachment to the arm if the spring is at given angle
    (angle is between 0 (up position) and travel_angle) """
    return codecad.util.Vector(math.cos(spring_up_angle - angle), math.sin(spring_up_angle - angle)) * spring_arm_length

def get_spring_anchor_point(spring_arm_length):
    """ Return the spring anchor point coordinates in 2D relative to arm pivot as codecad Vector.
    Spring is placed to be at right angle to the arm at full compression. """
    return get_spring_point(spring_arm_length, 0) + \
        codecad.util.Vector(-math.sin(spring_up_angle), math.cos(spring_up_angle)) * (spring_length - spring_travel)

def get_travel_angle(spring_arm_length, spring_anchor_point):
    """ Calculate travel angle of the suspension arm based on spring length.
    Uses spring anchor point placement from get_spring_anchor_point. """
    spring_compressed_length = spring_length - spring_travel

    if spring_arm_length == 0:
        return 0

    spring_anchor_distance = abs(spring_anchor_point)
    compressed_anchor_angle = math.atan(spring_compressed_length / spring_arm_length)

    tmp = (spring_arm_length**2 + spring_anchor_distance**2 - spring_length**2) / \
          (2 * spring_arm_length * spring_anchor_distance)
    return math.acos(tmp) - compressed_anchor_angle

def spring_arm_length_equation(spring_arm_length):
    """ Equation describing spring location relative to the pivot. """
    spring_anchor_point = get_spring_anchor_point(spring_arm_length)
    travel_angle = get_travel_angle(spring_arm_length, spring_anchor_point)
    spring_down_point = get_spring_point(spring_arm_length, travel_angle)

    spring_axis_to_pivot_point = point_to_line_distance(codecad.util.Vector(0, 0),
                                                        spring_anchor_point,
                                                        spring_down_point)

    return spring_axis_to_pivot_point - (arm_thickness / 2 + spring_diameter / 2 + arm_clearance)

spring_up_angle = -math.pi / 2
spring_arm_length = scipy.optimize.brentq(spring_arm_length_equation, 0, arm_thickness + spring_diameter)
spring_anchor_point = get_spring_anchor_point(spring_arm_length)
travel_angle = get_travel_angle(spring_arm_length, spring_anchor_point)

# Rotate spring position so that its anchor point is at the same level as pivot
spring_up_angle += math.acos(spring_anchor_point.x / abs(spring_anchor_point))
spring_anchor_point = get_spring_anchor_point(spring_arm_length)
spring_up_point = get_spring_point(spring_arm_length, 0)
spring_down_point = get_spring_point(spring_arm_length, travel_angle)

def get_wheel_force(arm_length, up_angle, angle):
    """ Return residual force on a group of wheels. """
    spring_point = get_spring_point(spring_arm_length, up_angle - angle)
    length = abs(spring_point - spring_anchor_point)
    spring_force = spring_preload_force + \
        (spring_full_compression_force - spring_preload_force) * (spring_length - length) / spring_travel

    torque = spring_force * point_to_line_distance(codecad.util.Vector(0, 0),
                                                   spring_anchor_point,
                                                   spring_point)
    wheel_force = torque / (arm_length * math.cos(angle))

    return wheel_force - parameters.design_weight / bogie_count

def get_arm_angle(arm_length, y):
    """ Calculate angle of the suspension arm at the top position """
    return math.asin(y / arm_length)

def get_arm_travel(arm_length, down_arm_angle, bogie_pivot_up_y):
    """ Calculate total length of wheel vertical travel """
    return bogie_pivot_up_y - math.sin(down_arm_angle) * arm_length

def get_bogie_wheel_position(angle, side):
    s = math.sin(angle)
    c = math.cos(angle)
    side *= bogie_wheel_spacing / 2
    return codecad.util.Vector(c * side + s * bogie_pivot_z,
                               s * side - c * bogie_pivot_z)

def bogie_pivot_up_y_equation(arm_length, bogie_pivot_up_y):
    up_angle = get_arm_angle(arm_length, bogie_pivot_up_y)
    down_angle = up_angle - travel_angle
    travel = get_arm_travel(arm_length, down_angle, bogie_pivot_up_y)

    neutral_angle = get_arm_angle(arm_length, bogie_pivot_up_y - (1 - suspension_sag) * travel)

    bogie_pivot_up_point = codecad.util.Vector(math.cos(up_angle), math.sin(up_angle)) * arm_length

    left_angle = (up_angle - neutral_angle) - bogie_swing_angle
    left_wheel_position = bogie_pivot_up_point + get_bogie_wheel_position(left_angle, -1)

    dist_left = point_to_line_distance(left_wheel_position, spring_up_point, spring_anchor_point)
    dist_right = abs(bogie_pivot_up_point - (spring_down_point + codecad.util.Vector(suspension_spacing, 0)))

    ret1 = dist_left - wheel_diameter / 2 - spring_diameter / 2 - wheel_clearance
    #ret2 = dist_right - math.hypot(bogie_wheel_spacing / 2, bogie_pivot_z) - wheel_diameter / 2 - arm_thickness / 2 - wheel_clearance

    return ret1
    #return min(ret1, ret2)

def get_optimized_bogie_pivot_up_y(arm_length):
    a = -arm_length
    b = arm_length

    va = bogie_pivot_up_y_equation(arm_length, a)
    vb = bogie_pivot_up_y_equation(arm_length, b)

    if (va > 0) == (vb > 0):
        if abs(va) < abs(vb):
            return a
        else:
            return b

    return scipy.optimize.brentq(lambda x: bogie_pivot_up_y_equation(arm_length, x),
                                 a, b)

def arm_length_equation(arm_length):
    bogie_pivot_up_y = get_optimized_bogie_pivot_up_y(arm_length)

    up_angle = get_arm_angle(arm_length, bogie_pivot_up_y)
    down_angle = up_angle - travel_angle
    travel = get_arm_travel(arm_length, down_angle, bogie_pivot_up_y)

    neutral_angle = get_arm_angle(arm_length, bogie_pivot_up_y - (1 - suspension_sag) * travel)

    ret = get_wheel_force(arm_length, up_angle, neutral_angle)
    return ret

arm_length = scipy.optimize.brentq(arm_length_equation,
                                   spring_length / 2, 3 * spring_length)
bogie_pivot_up_y = get_optimized_bogie_pivot_up_y(arm_length)

arm_up_angle = get_arm_angle(arm_length, bogie_pivot_up_y)
arm_down_angle = arm_up_angle - travel_angle
suspension_travel = get_arm_travel(arm_length, arm_down_angle, bogie_pivot_up_y)
arm_neutral_angle = get_arm_angle(arm_length,
                                  bogie_pivot_up_y - (1 - suspension_sag) * suspension_travel)

assert arm_down_angle > -math.pi / 2
assert suspension_travel >= suspension_min_travel
assert arm_length > spring_length - spring_travel
assert arm_up_angle - arm_neutral_angle < 2 * bogie_swing_angle
assert arm_neutral_angle - arm_down_angle < 2 * bogie_swing_angle
assert abs(bogie_pivot_up_y_equation(arm_length, bogie_pivot_up_y)) < wheel_clearance / 100, "Check that the bogie clearances are met"

def road_wheel_generator(diameter, width, axle_diameter,
                         shoulder_height, shoulder_width,
                         o_ring_minor_diameter, wall_thickness, hole_blinding_layer_height,
                         screw_hole_diameter, screw_hole_depth,
                         hex_hole):

    o_ring_protrusion = o_ring_minor_diameter / 2
    radius = diameter / 2 - o_ring_protrusion
    axle_radius = axle_diameter / 2

    wheel = polygon2d([(axle_radius, 0),
                       (radius, 0),
                       (radius, width),
                       (axle_radius + shoulder_width + shoulder_height, width),
                       (axle_radius + shoulder_width, width + shoulder_height),
                       (axle_radius, width + shoulder_height),
                       ])

    o_ring_count = 2
    o_ring_spacing = (width - o_ring_count * o_ring_minor_diameter) / (1 + o_ring_count)
    for i in range(o_ring_count):
        pos = (i + 1) * o_ring_spacing + (i + 0.5) * o_ring_minor_diameter
        wheel -= circle(d=o_ring_minor_diameter).translated(radius, pos)

    wheel = wheel.revolved().rotated_x(90)

    if hole_blinding_layer_height:
        wheel += cylinder(r=radius - o_ring_protrusion,
                          h=hole_blinding_layer_height,
                          symmetrical=False).translated_z(screw_hole_depth)

    if hex_hole:
        wheel -= regular_polygon2d(n=6, d=screw_hole_diameter * 2 / math.sqrt(3)).extruded(2 * screw_hole_depth)
    else:
        wheel -= cylinder(d=screw_hole_diameter, h=2 * screw_hole_depth)

    lightening_hole_count = 5
    lightening_hole_inner_radius = max(axle_radius + shoulder_height + shoulder_width,
                                       axle_radius + wall_thickness,
                                       screw_hole_diameter / 2 + wall_thickness)
    lightening_hole_outer_radius = radius - o_ring_protrusion - wall_thickness
    lightening_hole_center_radius = (lightening_hole_inner_radius + lightening_hole_outer_radius) / 2
    lightening_hole_polygon = regular_polygon2d(n=lightening_hole_count,
                                                r=lightening_hole_center_radius)
    lightening_hole_diameter = min(lightening_hole_outer_radius - lightening_hole_inner_radius,
                                   lightening_hole_polygon.side_length - wall_thickness)

    wheel -= cylinder(d=lightening_hole_diameter, h=float("inf")) \
        .translated_x(lightening_hole_center_radius) \
        .rotated((0, 0, 1), 360, n=lightening_hole_count)

    return wheel

def bogie_generator(wheel_spacing, lower_thickness, upper_thickness,
                    bearing_diameter, bearing_thickness, bearing_shoulder_size,
                    thin_wall, thick_wall,
                    pivot_z,
                    wheel_diameter,
                    arm_thickness,
                    arm_cutout_thickness,
                    arm_cutout_angle,
                    shoulder_screw_diameter,
                    shoulder_screw_diameter2,
                    shoulder_screw_length,
                    shoulder_screw_screw_length,
                    shoulder_screw_head_diameter,
                    shoulder_screw_head_height,
                    shoulder_screw_nut_height,
                    shoulder_screw_nut_s,
                    arm_knee_height,
                    arm_clearance,
                    weel_clearance):

    assert arm_cutout_angle < 180

    bearing_radius = bearing_diameter / 2
    bearing_shoulder_radius = bearing_radius - bearing_shoulder_size
    nut_outer_diameter = shoulder_screw_nut_s * 2 / math.sqrt(3)
    pivot_protected_diameter = max(shoulder_screw_diameter2, nut_outer_diameter)
    pivot_end_diameter = pivot_protected_diameter + 2 * thick_wall
    pivot_protected_diameter += 2 * thin_wall
    pivot_to_wheel_distance = math.hypot(wheel_spacing / 2, pivot_z)
    wheel_cutout_diameter = wheel_diameter + 2 * wheel_clearance
    wheel_cutout_angled_part = min(pivot_to_wheel_distance - wheel_cutout_diameter / 2 - pivot_protected_diameter / 2, (upper_thickness - lower_thickness) / 2)

    assert pivot_to_wheel_distance >= thin_wall + wheel_cutout_diameter / 2 + arm_thickness / 2
    assert arm_knee_height > pivot_end_diameter / 2 # This is a neccessary but not sufficient condition!

    bogie = polygon2d([(-wheel_spacing / 2, 0),
                       (wheel_spacing / 2, 0),
                       (0, pivot_z)]) \
        .offset((bearing_diameter + thin_wall + thick_wall) / 2) \
        .extruded(upper_thickness) \
        .rotated_x(90) \
        .translated_z((thick_wall - thin_wall) / 2)

    bottom_z = -bearing_radius - thin_wall

    bogie += cylinder(d=pivot_end_diameter, h=upper_thickness) \
        .rotated_x(90) \
        .translated_z(pivot_z)

    cutout_tmp_point = (wheel_spacing * math.sin(math.radians(arm_cutout_angle / 2)), wheel_spacing * math.cos(math.radians(arm_cutout_angle / 2)))
    screw_head_plane_y = -upper_thickness / 2 + shoulder_screw_head_height
    nut_plane_y = screw_head_plane_y + shoulder_screw_length - shoulder_screw_nut_height - thin_wall

    # Screw head
    bogie -= cylinder(d=shoulder_screw_head_diameter, h=upper_thickness, symmetrical=False) \
        .rotated_x(90) \
        .translated(0, screw_head_plane_y, pivot_z)
    # Smooth part
    bogie -= cylinder(d=shoulder_screw_diameter2, h=2 * (shoulder_screw_length - shoulder_screw_screw_length)) \
        .rotated_x(90) \
        .translated(0, screw_head_plane_y, pivot_z)
    # Screw part
    bogie -= cylinder(d=shoulder_screw_diameter, h=float("inf")) \
        .rotated_x(90) \
        .translated_z(pivot_z)
    # Nut
    bogie -= regular_polygon2d(n=6, d=nut_outer_diameter) \
        .rotated(30) \
        .extruded(upper_thickness, symmetrical=False) \
        .rotated_x(-90) \
        .translated(0, nut_plane_y, pivot_z)

    # Space for the arm
    cutout = polygon2d([(0, 0),
                        cutout_tmp_point,
                        (-cutout_tmp_point[0], cutout_tmp_point[1])]) \
        .offset(arm_thickness / 2)
    cutout += circle(d=arm_thickness + 2 * arm_clearance)
    bogie -= cutout \
        .extruded(lower_thickness) \
        .rotated_x(90) \
        .translated_z(pivot_z)

    # Wheel and bearing cutouts
    cutout = polygon2d([(-bearing_shoulder_radius, -upper_thickness),
                        (bearing_shoulder_radius, -upper_thickness),
                        (bearing_shoulder_radius, -bearing_thickness),
                        (bearing_radius, -bearing_thickness),
                        (bearing_radius, 0),
                        (wheel_cutout_diameter / 2, (upper_thickness - lower_thickness) / 2 - wheel_cutout_angled_part),
                        (wheel_cutout_diameter / 2 + (upper_thickness - lower_thickness) / 2 + wheel_cutout_angled_part, upper_thickness - lower_thickness),
                        (-bearing_shoulder_radius, upper_thickness - lower_thickness)])
    cutout += rectangle(wheel_diameter, 2 * upper_thickness) \
        .translated_y(upper_thickness + wheel_clearance) \
        .offset(wheel_clearance)

    cutout = cutout.translated_y(lower_thickness / 2)
    cutout = cutout + cutout.mirrored_y()

    cutout = cutout.revolved()
    for x in [-1, 1]:
        bogie -= cutout.translated_x(x * wheel_spacing / 2)

    # bottom lightening angles
    base_thickness = upper_thickness / 2 - (pivot_z - bottom_z - pivot_protected_diameter / 2)
    for y in [-1, 1]:
        bogie -= half_space().rotated_x(90 + y * 135).translated(0, -y * base_thickness, bottom_z)

    return bogie

def spring_cutout_generator(spring_angle, r0, r1, chamfer0, chamfer1=0):
    """ Make a shape that cuts out a space for the spring to move in.
    spring_angle is the angle that the spring will move relative to the cutout,
    r0 is the distance from origin that the cutout starts and r1 is distance from origin
    where it ends.
    chamfer0 is subtracted from r0 middle of the cutout height.
    chamfer1 is added to r1 in the middle of the cutout height"""

    assert 0 < spring_angle < 180

    r0 += arm_clearance
    r1 -= arm_clearance

    spring_r = spring_diameter / 2 + arm_clearance
    cos = math.cos(math.radians(spring_angle)) * 2 * r1
    sin = math.sin(math.radians(spring_angle)) * 2 * r1

    points1 = [(0, 0), (2 * r1, 0)]
    if spring_angle > 90:
        points1.append((2 * r1, 2 * r1))
    points1.append((cos, sin))

    points2 = [(0, 0), (0, -r1)]
    if spring_angle < 90:
        points2.append((-r1, 0))
    points2.append((-sin, cos))

    p = polygon2d(points1)

    s = p.extruded(0).offset(spring_r) + \
        p.offset(spring_r).extruded(spring_r, symmetrical=False) - \
        polygon2d(points2).offset(r0 - chamfer0).extruded(float("inf"))


    chamfer_poly = polygon2d([(r0 - chamfer0, 2 * spring_diameter),
                              (r0 - chamfer0, 0),
                              (r0, -spring_r),
                              (r1, -spring_r),
                              (r1 + chamfer1, 0),
                              (r1 + chamfer1, 2 * spring_diameter)])
    mask = chamfer_poly.revolved().rotated_x(90)
    mask &= p.extruded(float("inf"))
    mask += chamfer_poly.extruded(spring_diameter).translated_z(0.95 * spring_diameter / 2).rotated_x(90)
    mask += chamfer_poly.extruded(spring_diameter).translated_z(-0.95 * spring_diameter / 2).rotated_x(90).rotated_z(spring_angle)

    return s & mask

def arm_generator(thickness, pivot_thickness, width,
                  bogie_side_width,
                  arm_length, spring_arm_length,
                  arm_neutral_angle, arm_up_angle,
                  knee_height,
                  knee_angle,
                  pivot_mount_diameter, pivot_mount_height,
                  pivot_mount_screw_head_diameter, pivot_mount_screw_head_countersink,
                  spring_mount_diameter, spring_mount_height,
                  bogie_pivot_mount_diameter,
                  thin_wall, thick_wall,
                  hole_blinding_layer_height):
    spring_point_angle = spring_up_angle - arm_up_angle
    bogie_pivot = (arm_length, 0)
    spring_point = (spring_arm_length * math.cos(spring_point_angle), spring_arm_length * math.sin(spring_point_angle))

    pivot_outer_radius = pivot_thickness / 2

    knee_mid_angle = math.pi / 2 - arm_neutral_angle

    assert pivot_mount_height >= spring_mount_height

    knee_point1 = (bogie_pivot[0] + (knee_height + 0.2 * thickness) * math.cos(knee_mid_angle - math.radians(knee_angle / 2)),
                   bogie_pivot[1] + (knee_height + 0.2 * thickness) * math.sin(knee_mid_angle - math.radians(knee_angle / 2)))
    knee_point2 = (bogie_pivot[0] + knee_height * math.cos(knee_mid_angle + math.radians(knee_angle / 2)),
                   bogie_pivot[1] + knee_height * math.sin(knee_mid_angle + math.radians(knee_angle / 2)))
    # TODO: calculate the following angles instead of just pulling them out of thin air
    # (or use some better modelling tool instead of just offset polygons
    pivot_point1 = ((pivot_outer_radius - thickness / 2) * math.cos(math.radians(70)),
                    (pivot_outer_radius - thickness / 2) * math.sin(math.radians(70)))
    pivot_point2 = ((pivot_outer_radius - thickness / 2) * math.cos(math.radians(225)),
                    (pivot_outer_radius - thickness / 2) * math.sin(math.radians(225)))
    outline = polygon2d([(pivot_point1), knee_point1, bogie_pivot, knee_point2, spring_point, pivot_point2]) \
        .offset(thickness / 2)
    outline += circle(r=pivot_outer_radius)

    arm = outline.extruded(width + spring_mount_height, symmetrical=False)

    arm += tools.cone(height=pivot_mount_height - spring_mount_height,
                      upper_diameter=pivot_mount_diameter + 2 * thick_wall,
                      lower_diameter=2 * pivot_outer_radius,
                      base_height=width + spring_mount_height)

    spring_mount_top_diameter = thickness / 2
    spring_cutout_r0 = spring_mount_height + spring_mount_top_diameter / 2
    spring_down_vector = spring_anchor_point - spring_down_point
    rel_spring_down_angle = math.degrees(math.atan2(spring_down_vector.y, spring_down_vector.x) - arm_down_angle)
    arm -= spring_cutout_generator(90 + rel_spring_down_angle,
                                   spring_cutout_r0,
                                   2 * arm_length,
                                   (spring_diameter / 2 + arm_clearance)) \
        .rotated_z(-90) \
        .translated(spring_point[0], spring_point[1], width + spring_diameter / 2 + arm_clearance)

    holes = circle(d=pivot_mount_diameter) + \
        circle(d=bogie_pivot_mount_diameter).translated(*bogie_pivot) + \
        circle(d=spring_mount_diameter).translated(*spring_point)
    arm -= holes.extruded(float("inf"))

    # Pivot screw head countersink
    arm -= cylinder(d=pivot_mount_screw_head_diameter, h=2*pivot_mount_screw_head_countersink)

    if hole_blinding_layer_height:
        arm += cylinder(d=pivot_mount_screw_head_diameter,
                        h=hole_blinding_layer_height,
                        symmetrical=False).translated_z(pivot_mount_screw_head_countersink)

    return arm

def spring_placeholder_generator(length):
    spring = (capsule(0, 0, 0, length / 2, spring_top_mount_od) - circle(d=spring_top_mount_id)) \
        .extruded(spring_top_mount_thickness)
    spring += (capsule(0, length / 2, 0, length, spring_bottom_mount_od) - circle(d=spring_bottom_mount_id).translated_y(length)) \
        .extruded(spring_bottom_mount_thickness)
    spring = spring.rotated_x(90)
    spring += cylinder(d=spring_diameter, h=length * 2 / 3).translated_z(length * 0.45)
    return spring


inner_road_wheel = road_wheel_generator(wheel_diameter,
                                        half_wheel_width,
                                        vitamins.small_bearing.id,
                                        wheel_clearance,
                                        vitamins.small_bearing.shoulder_size,
                                        vitamins.o_ring.minor_diameter,
                                        4 * parameters.extrusion_width,
                                        parameters.layer_height,
                                        vitamins.small_screw.lock_nut.s,
                                        vitamins.small_screw.lock_nut.height + vitamins.small_screw.diameter / 6,
                                        True
                                        ).make_part("inner_road_wheel", ["3d_print"])
outer_road_wheel = road_wheel_generator(wheel_diameter,
                                        half_wheel_width,
                                        vitamins.small_bearing.id,
                                        wheel_clearance,
                                        vitamins.small_bearing.shoulder_size,
                                        vitamins.o_ring.minor_diameter,
                                        4 * parameters.extrusion_width,
                                        parameters.layer_height,
                                        vitamins.small_screw.head_diameter,
                                        vitamins.small_screw.head_height,
                                        False
                                        ).make_part("outer_road_wheel", ["3d_print"])
bogie = bogie_generator(bogie_wheel_spacing,
                        bogie_width, wheel_width,
                        vitamins.small_bearing.od, vitamins.small_bearing.thickness, vitamins.small_bearing.shoulder_size,
                        4 * parameters.extrusion_width, 6 * parameters.extrusion_width,
                        bogie_pivot_z,
                        wheel_diameter,
                        arm_thickness,
                        arm_width + pivot_flat_clearance,
                        bogie_arm_cutout_angle,
                        vitamins.shoulder_screw.diameter,
                        vitamins.shoulder_screw.diameter2,
                        vitamins.shoulder_screw.length,
                        vitamins.shoulder_screw.screw_length,
                        vitamins.shoulder_screw.head_diameter,
                        vitamins.shoulder_screw.head_height,
                        vitamins.shoulder_screw.lock_nut.height,
                        vitamins.shoulder_screw.lock_nut.s,
                        arm_knee_height,
                        arm_clearance,
                        wheel_clearance,
                        ).make_part("bogie", ["3d_print"])
arm_right = arm_generator(arm_thickness, vitamins.shoulder_screw.diameter2 + 24 * parameters.extrusion_width, arm_width,
                          arm_width - pivot_flat_clearance,
                          arm_length, spring_arm_length,
                          arm_neutral_angle, arm_up_angle,
                          arm_knee_height,
                          arm_knee_angle,
                          vitamins.shoulder_screw.diameter2 + pivot_round_clearance,
                          pivot_guide_length - arm_width + pivot_screw_head_countersink,
                          vitamins.shoulder_screw.head_diameter + pivot_flat_clearance,
                          pivot_screw_head_countersink,
                          spring_bottom_mount_id,
                          (spring_diameter - spring_bottom_mount_thickness) / 2 + arm_clearance,
                          vitamins.shoulder_screw.diameter2 + pivot_round_clearance,
                          3 * parameters.extrusion_width,
                          6 * parameters.extrusion_width,
                          parameters.layer_height,
                          ).make_part("suspension_arm_right", ["3d_print"])
arm_left = arm_right.shape().mirrored_x().make_part("suspension_arm_left", ["3d_print"])


bogie_assembly = codecad.assembly("bogie_assembly",
                                  [bogie.translated_z(wheel_diameter / 2),
                                   inner_road_wheel.rotated_x(90).translated(bogie_wheel_spacing / 2,
                                                                             wheel_width / 2,
                                                                             wheel_diameter / 2),
                                   inner_road_wheel.rotated_x(90).translated(-bogie_wheel_spacing / 2,
                                                                             wheel_width / 2,
                                                                             wheel_diameter / 2),
                                   outer_road_wheel.rotated_x(-90).translated(bogie_wheel_spacing / 2,
                                                                              -wheel_width / 2,
                                                                              wheel_diameter / 2),
                                   outer_road_wheel.rotated_x(-90).translated(-bogie_wheel_spacing / 2,
                                                                              -wheel_width / 2,
                                                                              wheel_diameter / 2)] +
                                   [vitamins.small_bearing] * 4 +
                                   [vitamins.small_screw, vitamins.small_screw.lock_nut] * 2 +
                                   [vitamins.o_ring] * 8
                                 )


# Y offset of a right suspension arm base in an assembly.
arm_base_offset = pivot_guide_length + pivot_screw_head_countersink

# Ofset for matching a piece of track with right suspension assembly
track_offset = codecad.util.Vector(arm_length * math.cos(arm_neutral_angle),
                                   arm_base_offset - arm_width / 2,
                                   arm_length * math.sin(arm_neutral_angle) - bogie_pivot_z - wheel_diameter / 2)

# Pivot mating surface is at coordinates 0, 0, 0 for both left and right arm

# Position of the matching surface for spring anchor point on the right side
# This one is rotated in print orientation!
spring_anchor_point = codecad.util.Vector(spring_anchor_point.x, spring_anchor_point.y,
                                          arm_base_offset - (arm_width + arm_clearance + spring_diameter / 2 + spring_top_mount_thickness / 2))


def suspension_generator(right, arm_angle = arm_neutral_angle, bogie_angle_fraction = None):
    spring_point = get_spring_point(spring_arm_length, arm_up_angle - arm_angle)

    v = spring_point - spring_anchor_point
    length = abs(v)

    spring_degrees = 90 - math.degrees(math.atan2(v.y, v.x))
    spring = spring_placeholder_generator(length).make_part("spring")

    degrees = -math.degrees(arm_angle)

    if bogie_angle_fraction is None:
        bogie_degrees = 0
    else:
        low = -bogie_swing_angle + (arm_angle - arm_neutral_angle)
        high = bogie_swing_angle + (arm_angle - arm_neutral_angle)
        bogie_degrees = low + (high - low) * bogie_angle_fraction
        bogie_degrees = math.degrees(bogie_degrees)

    if right:
        arm = arm_right
        bogie = bogie_assembly.rotated_z(180)
        multiplier = 1
    else:
        arm = arm_left.rotated_y(180)
        bogie = bogie_assembly
        multiplier = -1

    asm = codecad.assembly("suspension_assembly_" + ("right" if right else "left"),
                           [arm \
                                .rotated_x(90) \
                                .rotated_y(degrees) \
                                .translated_y(multiplier * arm_base_offset),
                            bogie \
                                .translated_z(-bogie_pivot_z - wheel_diameter / 2) \
                                .rotated_y(-degrees - bogie_degrees) \
                                .translated_x(arm_length) \
                                .rotated_y(degrees) \
                                .translated_y(multiplier * track_offset.y),
                            spring \
                                .rotated_y(spring_degrees) \
                                .translated(spring_anchor_point.x,
                                            multiplier * (spring_anchor_point.z + spring_top_mount_thickness / 2),
                                            spring_anchor_point.y),
                            vitamins.small_screw,
                            vitamins.small_screw.lock_nut,
                            vitamins.large_screw,
                            vitamins.large_screw.lock_nut,
                            vitamins.shoulder_screw,
                            vitamins.shoulder_screw,
                            vitamins.shoulder_screw.lock_nut,
                            vitamins.shoulder_screw.lock_nut,
                            ])

    return asm


suspension_assembly_left = suspension_generator(False)
suspension_assembly_right = suspension_generator(True)

if __name__ == "__main__":
    def p(name, f=lambda x: x):
        print(name, f(globals()[name]))

    p("arm_length")
    p("bogie_pivot_up_y")
    p("arm_up_angle", math.degrees)
    p("arm_neutral_angle", math.degrees)
    p("arm_down_angle", math.degrees)
    p("suspension_travel")

    #plot_wheel_forces(params)
    #codecad.commandline_render(suspension_generator(arm_neutral_angle, 0), 0.1)

    o = codecad.assembly("suspension_preview",
                         [suspension_assembly_left.translated_x(-suspension_spacing),
                          suspension_assembly_right,
                          suspension_assembly_left.translated_x(suspension_spacing)])
    o = suspension_assembly_left

    codecad.commandline_render(o, 0.1)
