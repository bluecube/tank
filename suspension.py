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

wheel_count = 10 # Count of road wheels on both sides of the vehicle

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

wheel_diameter = 30
wheel_width = 30 # Total width of the wheel pair
half_wheel_width = (wheel_width - track.guide_width) / 2 - track.clearance
p("half_wheel_width")

clearance = 2
arm_thickness = parameters.small_bearing_od + 2 * parameters.thick_wall

wheel_spacing = 60 # Distance between wheels [mm]

up_travel = 20 # [mm]
down_travel = 10 # [mm]

# Calculating suspension geometry:

assert 2 * track.guide_height + arm_thickness + 2 * track.clearance < wheel_diameter, \
       "Bearing must fit inside the wheel, accounting for track guide height"

# Variables for optimization:
ARM_LENGTH = 0
SPRING_ARM_LENGTH = 1
SPRING_ARM_ANGLE_OFFSET = 2 # Angle offset between wheel arm and spring arm
ARM_UP_ANGLE = 3
ARM_DOWN_ANGLE = 4
ARM_NEUTRAL_ANGLE = 5
SPRING_NEUTRAL_ANGLE = 6 # Angle between the spring arm and the spring in neutral position. 0 is along the spring arm
SPRING_NEUTRAL_LENGTH = 7 # Length of spring in neutral position

def polar2rect(angle, distance):
    return (math.cos(angle) * distance, math.sin(angle) * distance)

def dist(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def spring_anchor_point(p):
    spring_arm_angle = p[ARM_NEUTRAL_ANGLE] + p[SPRING_ARM_ANGLE_OFFSET]
    spring_angle = spring_arm_angle + p[SPRING_NEUTRAL_ANGLE]

    x1, y1 = polar2rect(spring_arm_angle, p[SPRING_ARM_LENGTH])
    x2, y2 = polar2rect(spring_angle, p[SPRING_NEUTRAL_LENGTH])

    return (x1 + x2, y1 + y2)

def lowest_point(p):
    """ Lowest allowed point on the chasis / suspension to avoid track interference
    at suspension up position """
    y = math.sin(p[ARM_UP_ANGLE]) * p[ARM_LENGTH] # Wheel center in up position
    y -= wheel_diameter / 2
    y += clearance
    return y

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

def suspension_width(p):
    """ Closest distance that the suspension elements can repeat after in X direction
    Includes clearance. """
    wheel_width = cosmax(p[ARM_UP_ANGLE], p[ARM_DOWN_ANGLE]) * p[ARM_LENGTH]
    wheel_width += wheel_diameter / 2

    spring_width = spring_anchor_point(p)[0]

    width = max(wheel_width, spring_width)
    width += arm_thickness / 2
    width += clearance

    return width

def suspension_height(p):
    """ Height over the wheel travel (!) that the suspension takes up.
    Might be zero (actually that is kind of the goal). """

    highest_point = math.sin(p[ARM_UP_ANGLE]) * p[ARM_LENGTH];
    highest_point += wheel_diameter / 2;

    pivot_height = -highest_point # Distance from the top of wheel travel to pivot in Y
    spring_height = spring_anchor_point(p)[1] - highest_point

    return max(pivot_height, spring_height, 0)


# The following functions must evaluate to zero
equalities = [lambda p: (math.sin(p[ARM_UP_ANGLE]) - math.sin(p[ARM_NEUTRAL_ANGLE])) * p[ARM_LENGTH] - up_travel, # Up travel distance
              lambda p: (math.sin(p[ARM_NEUTRAL_ANGLE]) - math.sin(p[ARM_DOWN_ANGLE])) * p[ARM_LENGTH] - down_travel, # Down travel distance
              lambda p: math.cos(p[ARM_NEUTRAL_ANGLE]) * p[ARM_LENGTH] * parameters.design_weight / wheel_count -
                        math.sin(p[SPRING_NEUTRAL_ANGLE]) * p[SPRING_ARM_LENGTH] * (spring_length - p[SPRING_NEUTRAL_LENGTH]) / spring_travel, # Neutral position torque
              lambda p: dist(spring_anchor_point(p), polar2rect(p[ARM_UP_ANGLE] + p[SPRING_ARM_ANGLE_OFFSET], p[SPRING_ARM_LENGTH])) - (spring_length - spring_travel), # Up position spring length
              lambda p: dist(spring_anchor_point(p), polar2rect(p[ARM_DOWN_ANGLE] + p[SPRING_ARM_ANGLE_OFFSET], p[SPRING_ARM_LENGTH])) - spring_length, # Down position spring length

              #lambda p: p[ARM_DOWN_ANGLE] + p[SPRING_ARM_ANGLE_OFFSET] + math.pi / 2, # Spring arm is exactly vertical below pivot
              ]
# The following functions must evaluate >= 0
inequalities = [lambda p: p[ARM_LENGTH], # Arm, lengths must be positive
                lambda p: p[SPRING_ARM_LENGTH], # -"-
                lambda p: spring_length - p[SPRING_NEUTRAL_LENGTH], # Neutral length of the spring must be shorter than max length
                lambda p: p[SPRING_NEUTRAL_LENGTH] - (spring_length - spring_travel), # Neutral length of the spring must be longer than min length
                lambda p: p[ARM_DOWN_ANGLE] + p[SPRING_ARM_ANGLE_OFFSET] + math.pi / 2, # Spring arm doesn't leave the X envelope in bottom position
                lambda p: spring_anchor_point(p)[1] - (lowest_point(p) + spring_diameter / 2), # Spring body won't interfere with tracks on the anchor point side at suspension up position
                lambda p: math.sin(p[ARM_UP_ANGLE] + p[SPRING_ARM_ANGLE_OFFSET]) * p[SPRING_ARM_LENGTH] -
                          (lowest_point(p) + spring_diameter / 2), # Spring body won't interfere with tracks on the arm side at suspension up position
                lambda p: -arm_thickness - lowest_point(p), # Pivot wont interfere with tracks at suspension up position
                lambda p: spring_anchor_point(p)[0], # Correct orientation of the spring
                ]

def objective(p):
    #print()
    #print(p)
    #for ineq in inequalities:
    #    print("    ", ineq(p))
    ret = suspension_width(p) ** 2 + suspension_height(p) ** 2
    #print(ret)
    return ret

initial = [0] * 8
initial[ARM_LENGTH] = wheel_diameter
initial[SPRING_ARM_LENGTH] = wheel_diameter * parameters.design_weight * down_travel / (wheel_count * spring_full_compression_force * (down_travel + up_travel))
initial[SPRING_ARM_ANGLE_OFFSET] = -math.pi / 4
initial[ARM_UP_ANGLE] = math.pi / 6
initial[ARM_DOWN_ANGLE] = -math.pi / 4
initial[ARM_NEUTRAL_ANGLE] = 0
initial[SPRING_NEUTRAL_ANGLE] = math.pi / 2
initial[SPRING_NEUTRAL_LENGTH] = spring_length - spring_travel * down_travel / (down_travel + up_travel)

print("Optimizing kinematics ...")
with warnings.catch_warnings():
    warnings.simplefilter("error")
    result = scipy.optimize.minimize(objective,
                                     initial,
                                     method="SLSQP",
                                     constraints=[{"type": "eq", "fun": fun} for fun in equalities] +
                                                 [{"type": "ineq", "fun": fun} for fun in inequalities])
params = result.x
print("done")


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
                         hub_width, hub_diameter,
                         o_ring_minor_diameter, wall_thickness,
                         screw_hole_diameter, screw_hole_depth,
                         hex_hole):

    o_ring_protrusion = o_ring_minor_diameter / 2
    radius = diameter / 2 - o_ring_protrusion
    axle_radius = axle_diameter / 2
    hub_radius = hub_diameter / 2

    rim_thickness = o_ring_minor_diameter / 2 + wall_thickness

    #wheel = polygon2d([(axle_radius, 0),
    #                  (axle_radius, hub_width),
    #                  (hub_radius, hub_width),
    #                  (hub_radius + hub_width - width / 2, width / 2),
    #                  (radius - rim_thickness - width / 2, width / 2),
    #                  (radius - rim_thickness, width),
    #                  (radius, width),
    #                  (radius, 0),
    #                  ])
    wheel = rectangle(radius - axle_radius, hub_width).translated_x(axle_radius + (radius - axle_radius) / 2)

    o_ring_count = 2
    o_ring_spacing = (width - o_ring_count * o_ring_minor_diameter) / (1 + o_ring_count)
    for i in range(o_ring_count):
        pos = (i + 1) * o_ring_spacing + (i + 0.5) * o_ring_minor_diameter
        wheel -= circle(d=o_ring_minor_diameter).translated(radius, pos)

    wheel = wheel.revolved().rotated_x(90)

    lightening_hole_count = 5
    lightening_hole_center_radius = radius / 2
    lightening_hole_polygon = regular_polygon2d(n=lightening_hole_count,
                                                r=lightening_hole_center_radius)
    lightening_hole_diameter = min(radius - rim_thickness - hub_radius - 2 * wall_thickness,
                                   lightening_hole_polygon.side_length - 2 * wall_thickness)

    wheel -= cylinder(d=lightening_hole_diameter, h=float("inf")) \
        .translated_x(lightening_hole_center_radius) \
        .rotated((0, 0, 1), 360, n=lightening_hole_count)

    return wheel


def spring_placeholder_generator(length): # Redo the geometry
    mounts = capsule(0, 0, 0, length, 8) - \
             circle(d=spring_bottom_mount_diameter) - \
             circle(d=spring_top_mount_diameter).translated_y(length)
    mounts = mounts.extruded(spring_top_mount_thickness).rotated_x(90)
    body = cylinder(d=spring_diameter, h=length * 0.7).translated_z(length * 0.55)
    return mounts + body


def suspension_generator(params, state):
    arm = generate_suspension_arm(params).make_part("suspension_arm", ["3d_print"])

    wheel =road_wheel_generator(wheel_diameter,
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

    anchor_point = spring_anchor_point(params)
    spring_length = dist(polar2rect(params[state] + params[SPRING_ARM_ANGLE_OFFSET], params[SPRING_ARM_LENGTH]),
                         anchor_point)
    spring = spring_placeholder_generator(spring_length).make_part("spring", ["vitamins"])

    degrees = -math.degrees(params[state])

    asm = codecad.Assembly([arm.rotated_x(90).rotated_y(degrees),
                            wheel.rotated_x(90).translated_x(params[ARM_LENGTH]).rotated_y(degrees),
                            spring.translated(anchor_point[0], 0, anchor_point[1])])

    return asm

if __name__ == "__main__":
    print(params)
    width = suspension_width(params)
    height = suspension_height(params)

    print("width, height: ", width, height)

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
