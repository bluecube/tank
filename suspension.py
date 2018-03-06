import matplotlib
matplotlib.rcParams['backend'] = "Qt5Agg"

import sys

import warnings
import math

import scipy.optimize

import codecad
from codecad.shapes import *

import util
import parameters
import track

def p(name, wrapper=lambda x: x):
    """ Just a helper to print global variable easily """
    print(name, wrapper(globals()[name]))

def trace(value, name=None):
    if name is None:
        print(value)
    else:
        print(name + ":", value)
    return value

bogie_count = 6 # Count of bogies on both sides of the vehicle

spring_length = 62 # Center to center, relaxed
spring_travel = 11
spring_diameter = 17.5
spring_top_mount_diameter = 5
spring_bottom_mount_diameter = 3
spring_top_mount_thickness = 3.8
spring_bottom_mount_thickness = 6.5
spring_preload_force = 0.95 # [kg]
spring_full_compression_force = 4.5 # [kg]

o_ring_minor_diameter = 2

arm_clearance = 1
clearance = 2

bogie_spacing = 110 # [mm] distance between bogies
bogie_wheel_spacing = 50 # [mm] distance between wheels of one bogie
wheel_diameter = 30
wheel_width = 30 # Total width of the wheel pair
arm_width = 8
bogie_width = arm_width

suspension_travel = 30 # [mm]
suspension_sag = 0.3 # Ratio of travel from neutral position down

half_wheel_width = (wheel_width - bogie_width) / 2 - arm_clearance

assert wheel_width - 2 * half_wheel_width >= track.guide_width + track.clearance, \
    "The track guide has enough space between the wheel halves"
assert wheel_width <= track.width

arm_thickness = parameters.shoulder_screw_diameter2 + 12 * parameters.extrusion_width


# Variables for optimization:
ARM_LENGTH = 0
SPRING_ARM_LENGTH = 1
SPRING_ARM_ANGLE_OFFSET = 2 # Angle offset between wheel arm and spring arm
ARM_UP_ANGLE = 3
ARM_DOWN_ANGLE = 4
ARM_NEUTRAL_ANGLE = 5
SPRING_ANCHOR_X = 6
SPRING_ANCHOR_Y = 7

def polar2rect(angle, distance):
    return (math.cos(angle) * distance, math.sin(angle) * distance)

def dist(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def cosmax(x1, x2):
    """ Maximum of cosine on a closed interval """
    x1, x2 = sorted([x1, x2])
    k1 = math.ceil(x1 / (2 * math.pi))
    k2 = math.floor(x2 / (2 * math.pi))
    if k1 <= k2:
        ret = 1
    else:
        ret = max(math.cos(x1), math.cos(x2))
    return ret

def lowest_point(p):
    """ Lowest allowed point on the chasis / suspension to avoid track interference
    at suspension up position """
    y = math.sin(p[ARM_UP_ANGLE]) * p[ARM_LENGTH] # Wheel center in up position
    y -= wheel_diameter / 2
    y += clearance
    return y

def spring_length_at_angle(p, angle):
    return dist(polar2rect(angle + p[SPRING_ARM_ANGLE_OFFSET], p[SPRING_ARM_LENGTH]),
                (p[SPRING_ANCHOR_X], p[SPRING_ANCHOR_Y]))

def distance_from_spring_to_point(p, angle, point):
    """ Calculate distance between spring axis and a point """
    spring_anchor_point = codecad.util.Vector(p[SPRING_ANCHOR_X], p[SPRING_ANCHOR_Y])
    spring_arm_point = codecad.util.Vector(*polar2rect(angle + p[SPRING_ARM_ANGLE_OFFSET],
                                                       p[SPRING_ARM_LENGTH]))
    v1 = (spring_anchor_point - spring_arm_point)
    length = abs(v1)
    v2 = codecad.util.Vector(v1.y, -v1.x) / length

    return (spring_anchor_point - codecad.util.Vector(*point)).dot(v2)

def wheel_force_at_angle(p, angle):
    """ Calculate upward force acting on the wheel at given suspension angle. """

    length = spring_length_at_angle(p, angle)

    spring_force = spring_preload_force + (spring_full_compression_force - spring_preload_force) * (spring_length - length) / spring_travel
    arm_torque = spring_force * distance_from_spring_to_point(p, angle, (0, 0))
    wheel_torque_distance = math.cos(p[ARM_NEUTRAL_ANGLE]) * p[ARM_LENGTH]
    wheel_force = arm_torque / wheel_torque_distance

    return wheel_force

def suspension_width(p):
    """ Closest distance that the suspension elements can repeat after in X direction
    Includes clearance. """
    wheel_width = cosmax(p[ARM_UP_ANGLE], p[ARM_DOWN_ANGLE]) * p[ARM_LENGTH]
    wheel_width += wheel_diameter / 2

    spring_width = p[SPRING_ANCHOR_X]

    width = max(wheel_width, spring_width)
    width += arm_thickness / 2
    width += clearance

    return width

def suspension_height(p):
    """ Height over the wheel travel (!) that the suspension takes up.
    Might be zero (actually that is kind of the goal). """

    wheel_height = math.sin(p[ARM_UP_ANGLE]) * p[ARM_LENGTH];
    wheel_height += wheel_diameter / 2;

    spring_height = p[SPRING_ANCHOR_Y]

    height = max(wheel_height, spring_height)
    height += arm_thickness / 2

    height -= math.sin(p[ARM_NEUTRAL_ANGLE]) * p[ARM_LENGTH]

    return height


# The following functions must evaluate to zero
equalities = [lambda p: 1000 * (wheel_force_at_angle(p, p[ARM_NEUTRAL_ANGLE]) - parameters.design_weight / wheel_count), # Neutral position torque
              lambda p: dist((p[SPRING_ANCHOR_X], p[SPRING_ANCHOR_Y]), polar2rect(p[ARM_UP_ANGLE] + p[SPRING_ARM_ANGLE_OFFSET], p[SPRING_ARM_LENGTH])) - (spring_length - spring_travel), # Up position spring length
              lambda p: dist((p[SPRING_ANCHOR_X], p[SPRING_ANCHOR_Y]), polar2rect(p[ARM_DOWN_ANGLE] + p[SPRING_ARM_ANGLE_OFFSET], p[SPRING_ARM_LENGTH])) - spring_length, # Down position spring length
              lambda p: 100 * ((math.sin(p[ARM_NEUTRAL_ANGLE]) - math.sin(p[ARM_DOWN_ANGLE])) / (math.sin(p[ARM_UP_ANGLE]) - math.sin(p[ARM_DOWN_ANGLE])) - suspension_sag), # Suspension sag ratio is as selected

              #lambda p: p[ARM_DOWN_ANGLE] + p[SPRING_ARM_ANGLE_OFFSET] + math.pi / 2, # Spring arm is exactly vertical below pivot
              ]
# The following functions must evaluate >= 0
inequalities = [lambda p: p[ARM_LENGTH], # Arm, lengths must be positive
                lambda p: p[SPRING_ARM_LENGTH], # -"-
                lambda p: p[ARM_DOWN_ANGLE] + math.radians(70), # Lower than -90 degrees angle makes no sense
                lambda p: math.pi / 2 - p[ARM_UP_ANGLE], # Higher than +90 degrees angle makes no sense
                lambda p: p[ARM_NEUTRAL_ANGLE] - p[ARM_DOWN_ANGLE], # Arm must turn in one direction
                lambda p: p[ARM_UP_ANGLE] - p[ARM_NEUTRAL_ANGLE], # -"-
                lambda p: (math.sin(p[ARM_UP_ANGLE]) - math.sin(p[ARM_DOWN_ANGLE])) * p[ARM_LENGTH] - suspension_travel, # suspension travel distance is at least the selected one
                lambda p: spring_length - spring_length_at_angle(p, p[ARM_NEUTRAL_ANGLE]), # Neutral length of the spring must be shorter than max length
                lambda p: spring_length_at_angle(p, p[ARM_NEUTRAL_ANGLE]) - (spring_length - spring_travel), # Neutral length of the spring must be longer than min length
                #lambda p: p[ARM_DOWN_ANGLE] + p[SPRING_ARM_ANGLE_OFFSET] + math.pi / 2, # Spring arm doesn't leave the X envelope in bottom position
                lambda p: p[SPRING_ANCHOR_Y] - (lowest_point(p) + spring_diameter / 2), # Spring body won't interfere with tracks on the anchor point side at suspension up position
                lambda p: math.sin(p[ARM_UP_ANGLE] + p[SPRING_ARM_ANGLE_OFFSET]) * p[SPRING_ARM_LENGTH] -
                          (lowest_point(p) + spring_diameter / 2), # Spring body won't interfere with tracks on the arm side at suspension up position
                lambda p: -arm_thickness - lowest_point(p), # Pivot wont interfere with tracks at suspension up position
                lambda p: p[SPRING_ANCHOR_X], # Correct orientation of the spring
                lambda p: wheel_force_at_angle(p, p[ARM_UP_ANGLE]) - wheel_force_at_angle(p, p[ARM_NEUTRAL_ANGLE]), # Force increases through the travel 1
                lambda p: wheel_force_at_angle(p, p[ARM_NEUTRAL_ANGLE]) - wheel_force_at_angle(p, p[ARM_DOWN_ANGLE]), # Force increases through the travel 2

                lambda p: distance_from_spring_to_point(p, p[ARM_UP_ANGLE], (0, 0)) - (spring_diameter / 2 + arm_thickness) / 2,
                lambda p: distance_from_spring_to_point(p, p[ARM_NEUTRAL_ANGLE], (0, 0)) - (spring_diameter / 2 + arm_thickness) / 2,
                lambda p: distance_from_spring_to_point(p, p[ARM_DOWN_ANGLE], (0, 0)) - (spring_diameter / 2 + arm_thickness) / 2,
                ]

def objective(p):
    #print()
    #print(p)
    #for ineq in inequalities:
    #    print("    ", ineq(p))
    ret = suspension_width(p) ** 2 + 0.25 * suspension_height(p) ** 2
    #print(ret)
    return ret

initial = [0] * 8
initial[ARM_LENGTH] = wheel_diameter
initial[SPRING_ARM_LENGTH] = wheel_diameter * parameters.design_weight * suspension_sag / (bogie_count * spring_full_compression_force)
initial[SPRING_ARM_ANGLE_OFFSET] = -math.pi / 4
initial[ARM_UP_ANGLE] = math.pi / 6
initial[ARM_DOWN_ANGLE] = -math.pi / 4
initial[ARM_NEUTRAL_ANGLE] = 0
initial[SPRING_ANCHOR_X] = spring_length
initial[SPRING_ANCHOR_Y] = spring_length / 5

all_constraints = [{"type": "eq", "fun": fun} for fun in equalities] + \
                  [{"type": "ineq", "fun": fun} for fun in inequalities]

#print("Optimizing kinematics - step 1 ...")
#with warnings.catch_warnings():
#    warnings.simplefilter("error")
#    #result = scipy.optimize.minimize(objective,
#    #                                 initial,
#    #                                 method="SLSQP",
#    #                                 options={"maxiter": 5000},
#    #                                 constraints=[{"type": "eq", "fun": fun} for fun in equalities] +
#    #                                             [{"type": "ineq", "fun": fun} for fun in inequalities])
#    result = scipy.optimize.minimize(objective,
#                                     initial,
#                                     method="COBYLA",
#                                     options={"maxiter": 50000,
#                                              "rhobeg": 1,
#                                              "catol": 1},
#                                     constraints=[{"type": "ineq", "fun": fun} for fun in equalities] +
#                                                 [{"type": "ineq", "fun": lambda p, fun=fun: -fun(p)} for fun in equalities] +
#                                                 [{"type": "ineq", "fun": fun} for fun in inequalities])
#    print(result)
#params = result.x
#print("done")
params = initial

def plot_wheel_forces(params):
    import matplotlib.pyplot as plt
    import numpy

    angles = numpy.linspace(params[ARM_DOWN_ANGLE], params[ARM_UP_ANGLE], 200)
    heights = (numpy.sin(angles) - numpy.sin(params[ARM_NEUTRAL_ANGLE])) * params[ARM_LENGTH]
    forces = numpy.vectorize(lambda angle: wheel_force_at_angle(params, angle))(angles)

    plt.plot(heights, forces)
    plt.xlabel("suspension height")
    plt.ylabel("suspension force")
    plt.grid()
    plt.show()

def generate_suspension_arm(params):
    wheel_mount = (params[ARM_LENGTH], 0)
    spring_mount = (math.cos(params[SPRING_ARM_ANGLE_OFFSET]) * params[SPRING_ARM_LENGTH],
                    math.sin(params[SPRING_ARM_ANGLE_OFFSET]) * params[SPRING_ARM_LENGTH])

    outline = polygon2d([(0, 0), wheel_mount, spring_mount]).offset(arm_thickness / 2)

    outline -= circle(d=parameters.small_bearing_id).translated(*wheel_mount)
    outline -= circle(d=spring_bottom_mount_diameter).translated(*spring_mount)

    arm = outline.extruded(arm_thickness) #!!!!! wrong dimension

    return arm

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
                    wheel_cutout_diameter,
                    arm_cutout_diameter,
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
                    overhang_angle):

    assert arm_cutout_angle < 180

    bogie = polygon2d([(-wheel_spacing / 2, 0),
                       (wheel_spacing / 2, 0),
                       (0, pivot_z)]) \
        .offset((bearing_diameter + thin_wall + thick_wall) / 2) \
        .extruded(upper_thickness) \
        .rotated_x(90) \
        .translated_z((bearing_diameter + thin_wall + thick_wall) / 2)

    wheel_axis_z = bearing_diameter / 2 + thin_wall
    pivot_z += wheel_axis_z

    nut_outer_diameter = shoulder_screw_nut_s * 2 / math.sqrt(3)
    pivot_end_diameter = max(shoulder_screw_diameter2, nut_outer_diameter) + 2 * thick_wall

    bogie += cylinder(d=pivot_end_diameter, h=upper_thickness) \
        .rotated_x(90) \
        .translated_z(pivot_z)

    cutout_tmp_point = (wheel_spacing * math.sin(math.radians(arm_cutout_angle / 2)), wheel_spacing * math.cos(math.radians(arm_cutout_angle / 2)))
    screw_sink_in = -2#(upper_thickness - (shoulder_screw_length + shoulder_screw_head_height)) / 2
    screw_head_plane_y = - (shoulder_screw_length - shoulder_screw_screw_length) / 2
    nut_plane_y = screw_head_plane_y + shoulder_screw_length - shoulder_screw_nut_height - shoulder_screw_diameter / 6

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
        .extruded(upper_thickness, symmetrical=False) \
        .rotated_x(-90) \
        .translated(0, nut_plane_y, pivot_z)

    # Space for the arm
    bogie -= polygon2d([(0, 0),
                        cutout_tmp_point,
                        (-cutout_tmp_point[0], cutout_tmp_point[1])]) \
        .offset(arm_cutout_diameter / 2) \
        .extruded(lower_thickness) \
        .rotated_x(90) \
        .translated_z(pivot_z)

    # Wheel cutouts
    for x in [-1, 1]:
        x = x * wheel_spacing / 2
        bogie -= cylinder(d=bearing_diameter - 2 * bearing_shoulder_size, h=lower_thickness).rotated_x(90).translated(x, 0, wheel_axis_z)
        for y in [-1, 1]:
            bogie -= cylinder(d=wheel_cutout_diameter, h=upper_thickness - lower_thickness + 1e-2).rotated_x(90).translated(x, y * upper_thickness / 2, wheel_axis_z)
            bogie -= cylinder(d=bearing_diameter, h=2 * bearing_thickness).rotated_x(90).translated(x, y * lower_thickness / 2, wheel_axis_z)

    # bottom lightening angles
    for y in [-1, 1]:
        bogie -= half_space().rotated_x(90 + y * (90 + overhang_angle)).translated_y(-y * lower_thickness / 2)

    return bogie

def spring_placeholder_generator(length): # Redo the geometry
    mounts = capsule(0, 0, 0, length, 8) - \
             circle(d=spring_bottom_mount_diameter) - \
             circle(d=spring_top_mount_diameter).translated_y(length)
    mounts = mounts.extruded(spring_top_mount_thickness).rotated_x(90)
    body = cylinder(d=spring_diameter, h=length * 0.7).translated_z(length * 0.55)
    return mounts + body


arm = generate_suspension_arm(params).make_part("suspension_arm", ["3d_print"])
inner_wheel = road_wheel_generator(wheel_diameter,
                                   half_wheel_width,
                                   parameters.small_bearing_id,
                                   arm_clearance,
                                   parameters.small_bearing_shoulder_size,
                                   o_ring_minor_diameter,
                                   4 * parameters.layer_height,
                                   parameters.layer_height,
                                   parameters.small_screw_nut_s,
                                   parameters.small_screw_nut_height + parameters.small_screw_diameter / 6,
                                   True).make_part("inner_road_wheel", ["3d_print"])
outer_wheel = road_wheel_generator(wheel_diameter,
                                   half_wheel_width,
                                   parameters.small_bearing_id,
                                   arm_clearance,
                                   parameters.small_bearing_shoulder_size,
                                   o_ring_minor_diameter,
                                   4 * parameters.layer_height,
                                   parameters.layer_height,
                                   parameters.small_screw_head_diameter,
                                   parameters.small_screw_head_height,
                                   False).make_part("outer_road_wheel", ["3d_print"])
bogie = bogie_generator(bogie_wheel_spacing,
                        bogie_width, parameters.shoulder_screw_length + parameters.shoulder_screw_head_height,
                        parameters.small_bearing_od, parameters.small_bearing_thickness, parameters.small_bearing_shoulder_size,
                        4 * parameters.layer_height, 6 * parameters.layer_height,
                        12, # Pivot Z
                        wheel_diameter + 2 * clearance,
                        arm_thickness + 2 * arm_clearance,
                        arm_width,
                        120, # Arm cutout angle
                        parameters.shoulder_screw_diameter,
                        parameters.shoulder_screw_diameter2,
                        parameters.shoulder_screw_length,
                        parameters.shoulder_screw_screw_length,
                        parameters.shoulder_screw_head_diameter,
                        parameters.shoulder_screw_head_height,
                        parameters.shoulder_screw_nut_height,
                        parameters.shoulder_screw_nut_s,
                        parameters.overhang_angle,
                        ).make_part("bogie", ["3d_print"])


def suspension_generator(params, state):
    arm_point = polar2rect(params[state] + params[SPRING_ARM_ANGLE_OFFSET], params[SPRING_ARM_LENGTH])
    anchor_point = (params[SPRING_ANCHOR_X], params[SPRING_ANCHOR_Y])

    v = (arm_point[0] - anchor_point[0], arm_point[1] - anchor_point[1])

    spring_length = math.hypot(v[0], v[1])
    spring_degrees = 90 - math.degrees(math.atan2(v[1], v[0]))
    spring = spring_placeholder_generator(spring_length).make_part("spring", ["vitamins"])

    degrees = -math.degrees(params[state])

    asm = codecad.Assembly([arm.rotated_x(90).rotated_y(degrees),
                            wheel.rotated_x(90).translated_x(params[ARM_LENGTH]).rotated_y(degrees),
                            spring.rotated_y(spring_degrees).translated(params[SPRING_ANCHOR_X], 15, params[SPRING_ANCHOR_Y])])

    return asm


if __name__ == "__main__":
    codecad.commandline_render(bogie.shape(), 0.1)
    sys.exit()
if __name__ == "__main__":
    print(params)
    width = suspension_width(params)
    height = suspension_height(params)

    print("width, height:", width, height)
    print("up travel:", (math.sin(params[ARM_UP_ANGLE]) - math.sin(params[ARM_NEUTRAL_ANGLE])) * params[ARM_LENGTH])
    print("down travel:", (math.sin(params[ARM_NEUTRAL_ANGLE]) - math.sin(params[ARM_DOWN_ANGLE])) * params[ARM_LENGTH])

    plot_wheel_forces(params)

    s1 = suspension_generator(params, ARM_DOWN_ANGLE).make_part("down_suspension")
    s2 = suspension_generator(params, ARM_NEUTRAL_ANGLE).make_part("neutral_suspension")
    s3 = suspension_generator(params, ARM_UP_ANGLE).make_part("up_suspension")

    o = codecad.Assembly([s1.translated_x(-width),
                          s2,
                          s3.translated_x(width)])

    codecad.commandline_render(o, 0.1)

sys.exit()













































inner_road_wheel = road_wheel_generator(wheel_diameter,
                                        wheel_width,
                                        parameters.small_bearing_id,
                                        wheel_width * 0.75,
                                        parameters.small_screw_nut_diameter + parameters.thin_wall,
                                        o_ring_minor_diameter,
                                        parameters.thin_wall,
                                        parameters.small_screw_nut_diameter,
                                        parameters.small_screw_nut_height,
                                        True,
                                        ).make_part("inner_road_wheel", ["3d_print"])
outer_road_wheel = road_wheel_generator(wheel_diameter,
                                        wheel_width,
                                        parameters.small_bearing_id,
                                        wheel_width * 0.75,
                                        parameters.small_screw_nut_diameter + parameters.thin_wall,
                                        o_ring_minor_diameter,
                                        parameters.thin_wall,
                                        parameters.small_screw_head_diameter,
                                        parameters.small_screw_head_height,
                                        False,
                                        ).make_part("outer_road_wheel", ["3d_print"])

left_suspension_arm = suspension_arm_generator(False,
                                               track.guide_width,
                                               arm_thickness,
                                               parameters.small_bearing_od,
                                               parameters.small_bearing_thickness,
                                               parameters.small_bearing_shoulder_size,
                                               arm_pivot_dx, arm_pivot_dy, 5,
                                               spring_bottom_dx, spring_bottom_dy, spring_bottom_mount_diameter,
                                               parameters.layer_height)


if __name__ == "__main__":
    codecad.commandline_render(left_suspension_arm.shape().rotated_x(45), 0.1)

sys.exit()

spring_placeholder = spring_placeholder_generator(parameters.spring_length,
                                                  parameters.spring_diameter,
                                                  parameters.spring_top_mount_diameter,
                                                  parameters.spring_bottom_mount_diameter,
                                                  parameters.spring_mount_thickness
                                                  ).make_part("60mm_shock_absorber", ["buy"])

def make_wheel_suspension(right):
    name = "right" if right else "left"
    direction = -1 if right else 1

    arm_length = math.hypot(parameters.suspension_arm_dx, parameters.suspension_arm_dy)

    assert parameters.spring_bottom_mount_diameter == parameters.small_screw_diameter
    assert arm_length + parameters.road_wheel_diameter / 2 + \
           parameters.small_screw_head_diameter / 2 < parameters.road_wheel_base_spacing, \
           "Two wheels on the same side will interfere during full travel"
    assert (arm_length - parameters.suspension_arm_dx) * 2 + \
           parameters.road_wheel_diameter < parameters.road_wheel_base_spacing, \
           "Two wheels on the opposing side will interfere during full travel"

    suspension_arm = suspension_arm_generator(right,
                                              parameters.suspension_arm_dx,
                                              parameters.suspension_arm_dy,
                                              parameters.suspension_arm_thickness,
                                              parameters.suspension_arm_height,
                                              parameters.small_screw_diameter,
                                              parameters.suspension_spring_angle,
                                              parameters.suspension_arm_wheel_clearance + parameters.road_wheel_inner_inset,
                                              parameters.small_bearing_id,
                                              parameters.small_bearing_shoulder_size
                                              ).make_part("{}_suspension_arm".format(name), ["3d_print"])

    return codecad.Assembly([road_wheel.rotated_x(90),
                             suspension_arm.rotated_x(90) \
                                 .translated_y(parameters.road_wheel_width / 2 +
                                               parameters.road_wheel_arm_clearance +
                                               parameters.suspension_arm_thickness),
                             spring_placeholder \
                                 .rotated_y(direction * parameters.suspension_spring_angle) \
                                 .translated_y(parameters.road_wheel_width / 2 +
                                               parameters.road_wheel_arm_clearance +
                                               parameters.suspension_arm_thickness +
                                               parameters.spring_mount_thickness / 2)
                             ]).make_part("{}_suspension".format(name))


left_wheel_suspension = make_wheel_suspension(False)
right_wheel_suspension = make_wheel_suspension(True)

if __name__ == "__main__":
    codecad.commandline_render(left_wheel_suspension, 0.1)
