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

wheel_count = 8 # Count of road wheels on both sides of the vehicle

spring_length = 62 # Center to center, relaxed
spring_travel = 11
spring_diameter = 17.5
spring_top_mount_diameter = 5
spring_bottom_mount_diameter = 3
spring_top_mount_thickness = 3.8
spring_bottom_mount_thickness = 6.5
spring_full_compression_force = 4 # [kg]

o_ring_minor_diameter = 2

wheel_diameter = 30
wheel_width = 30 # Total width of the wheel pair
half_wheel_width = (wheel_width - track.guide_width) / 2 - track.clearance
p("half_wheel_width")

arm_clearance = 2
arm_thickness = parameters.small_bearing_od + 2 * parameters.thick_wall

wheel_spacing = 60 # Distance between wheels [mm]

up_travel = 20 # [mm]
down_travel = 10 # [mm]

# Calculating suspension geometry:

assert 2 * track.guide_height + arm_thickness + 2 * track.clearance < wheel_diameter, \
       "Bearing must fit inside the wheel, accounting for track guide height"

# First select arm length and pivot point offsets according to full-stretch conditions

# Arm length selected so that neighboring wheels don't collide at full extension:
arm_length = wheel_spacing - wheel_diameter / 2 - arm_thickness / 2 - arm_clearance
p("arm_length")

# Arm pivot dx and dy are positions of the arm pivot points relative to neutral wheel position
# # dx selected so that inner (opposite side) neighbors don't collide at full extension:
# arm_pivot_dx = arm_length + (wheel_diameter + arm_clearance - wheel_spacing) / 2
# arm_pivot_dx = min(arm_pivot_dx, arm_length) # We could accidentaly make the arm longer than it really is
# arm_pivot_dx = -arm_pivot_dx
# selected so that the arm is horizontal in neutral position (TODO)
arm_pivot_dx = arm_length
arm_pivot_dy = 0
p("arm_pivot_dx")
p("arm_pivot_dy")

# Convert arm dx and dy to angles of neutral, lower and upper suspension positions
neutral_arm_angle = math.atan(arm_pivot_dy / arm_pivot_dx)
down_arm_angle = math.asin((-down_travel - arm_pivot_dy) / arm_length)
up_arm_angle = math.asin((up_travel - arm_pivot_dy) / arm_length)

p("neutral_arm_angle", math.degrees)
p("down_arm_angle", math.degrees)
p("up_arm_angle", math.degrees)

spring_relative_angle = -math.pi / 2 - down_arm_angle
p("spring_relative_angle", math.degrees)
#spring_relative_angle = 0

# Spring location is rather complicated, we will use scipy.fsolve for that
# Quite a lot of pen and paper pre-chewing was necessary to get it to only a single
# variable :-)

spring_compressed_length = spring_length - spring_travel
arm_torque_wheel = arm_length * math.cos(neutral_arm_angle) * parameters.design_weight / wheel_count

def spring_bottom_pivot(spring_arm_length, arm_angle):
    spring_arm_angle = arm_angle + spring_relative_angle
    return spring_arm_length * math.cos(spring_arm_angle), spring_arm_length * math.sin(spring_arm_angle)


def spring_mount_point(spring_arm_length):
    """ Returns spring mount position (top pivot) given spring arm length.
    Uses the global variables calculated earlier """

    if spring_arm_length <= 0:
        raise ValueError("Negative arm length is not ok")

    x0, y0 = spring_bottom_pivot(spring_arm_length, down_arm_angle)
    x2, y2 = spring_bottom_pivot(spring_arm_length, up_arm_angle)


    alpha = up_arm_angle - down_arm_angle # Angle between extreme arm positions
    t = 2 * spring_arm_length * math.sin(alpha / 2) # Distance between lower spring mounts at extreme arm positions

    # Epsilon is the angle between spring in top position and its arm.
    # This is obtained by imagining a fixed spring_arm_length,  drawing bottom mounts
    # of the spring in both extreme positions and constructing a triangle with the two
    # states of the spring as two sides and connection between the lower spring mount
    # positions as the third side.
    # Epsilon then comes out when solving angles in these triangles.
    epsilon = math.acos((t**2 + spring_compressed_length**2 - spring_length**2) / (2 * t * spring_compressed_length)) - (math.pi - alpha) / 2

    # Now that we know epsilon, we can calculate spring top mount positions
    x = x2 + spring_compressed_length * math.cos(up_arm_angle + spring_relative_angle + math.pi - epsilon)
    y = y2 + spring_compressed_length * math.sin(up_arm_angle + spring_relative_angle + math.pi - epsilon)

    # Just checking ...
    assert math.hypot(x - x2, y - y2) == util.Approx(spring_compressed_length)
    assert math.hypot(x - x0, y - y0) == util.Approx(spring_length)

    return x, y


def torque_error(spring_arm_length):
    spring_arm_length = spring_arm_length[0] # fsolve only deals with arrays

    try:
        x1, y1 = spring_mount_point(spring_arm_length)
    except ValueError:
        #print("{} is invalid spring arm length".format(spring_arm_length))
        return 1e6
    x2, y2 = spring_bottom_pivot(spring_arm_length, neutral_arm_angle)
    dx = x2 - x1
    dy = y2 - y1

    spring_neutral_length = math.hypot(dx, dy)
    dx /= spring_neutral_length
    dy /= spring_neutral_length
    # Effective arm length is distance from point 0, 0 to the spring center line in neutral position
    tmp = x1 * dx + y1 * dy
    effective_arm_length = math.hypot(x1 - tmp * dx, y1 - tmp * dy)

    arm_torque_spring = effective_arm_length * spring_full_compression_force * (spring_length - spring_neutral_length) / spring_travel

    error = arm_torque_spring - arm_torque_wheel
    #print("torque error for arm length {} is {} - {} = {}".format(spring_arm_length, arm_torque_spring, arm_torque_wheel, error))
    return error


# Initial guess for spring arm length would be exact if the spring was perpendicular to
# the spring arm at neutral position and the wheel moved only straight up/down

guess_spring_force = (down_travel * spring_full_compression_force / (up_travel + down_travel))
guess_arm_length = arm_torque_wheel / guess_spring_force
print("Optimizing spring arm length ...")
with warnings.catch_warnings():
    warnings.simplefilter("error")
    spring_arm_length = scipy.optimize.fsolve(torque_error, guess_arm_length)[0]

print("final torque error", torque_error([spring_arm_length]))
p("spring_arm_length")

spring_mount_dx, spring_mount_dy = spring_mount_point(spring_arm_length)
spring_mount_dx += arm_pivot_dx
spring_mount_dy += arm_pivot_dy
p("spring_mount_dx")
p("spring_mount_dy")

spring_bottom_dx, spring_bottom_dy = spring_bottom_pivot(spring_arm_length, neutral_arm_angle)
spring_bottom_dx += arm_pivot_dx
spring_bottom_dy += arm_pivot_dy
p("spring_bottom_dx")
p("spring_bottom_dy")

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

    wheel = polygon2d([(axle_radius, 0),
                      (axle_radius, hub_width),
                      (hub_radius, hub_width),
                      (hub_radius + hub_width - width / 2, width / 2),
                      (radius - rim_thickness - width / 2, width / 2),
                      (radius - rim_thickness, width),
                      (radius, width),
                      (radius, 0),
                      ])

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


def suspension_arm_generator(flip,
                             thickness, height,
                             bearing_od, bearing_thickness, bearing_shoulder,
                             pivot_dx, pivot_dy, pivot_diameter,
                             spring_dx, spring_dy, spring_mount_diameter,
                             support_layer_height):
    # Position 0, 0 is the wheel center

    if flip:
        pivot_dx = -pivot_dx
        spring_dx = -spring_dx

    assert height > bearing_od
    assert thickness > 2 * bearing_thickness

    arm = union([capsule(0, 0, spring_dx, spring_dy, height),
                 capsule(spring_dx, spring_dy, pivot_dx, pivot_dy, height)]) # TODO: rounding
    arm = polygon2d([(0, 0),
                     (spring_dx, spring_dy),
                     (pivot_dx, pivot_dy)]).offset(height / 2)

    arm = arm.extruded(thickness, symmetrical=False)

    #arm += polygon2d([(0, 0), (0, knob_height),
    #                  (bearing_id / 2 + bearing_shoulder_size, knob_height),
    #                  (height / 2, 0)]) \
    #    .revolved() \
    #    .rotated_x(90) \
    #    .translated_z(thickness)

    bearing_hole = cylinder(d=bearing_od, h=2 * bearing_thickness)
    arm -= bearing_hole
    arm -= bearing_hole.translated_z(thickness)

    bearing_inner_hole = cylinder(d=bearing_od - 2 * bearing_shoulder,
                                  h=thickness - 2 * bearing_thickness - support_layer_height)
    arm -= bearing_inner_hole.translated_z(thickness - bearing_thickness)

    return arm


def spring_placeholder_generator(length, diameter, top_mount_diameter, bottom_mount_diameter,
                                 mount_thickness):
    mounts = capsule(0, 0, 0, length, 8) - \
             circle(d=bottom_mount_diameter) - \
             circle(d=top_mount_diameter).translated_y(length)
    mounts = mounts.extruded(mount_thickness).rotated_x(90)
    body = cylinder(d=diameter, h=length * 0.7).translated_z(length * 0.55)
    return mounts + body

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
