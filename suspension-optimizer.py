import matplotlib
matplotlib.rcParams['backend'] = "Qt5Agg"

import sys

import warnings
import math

import scipy.optimize

import util
import parameters
import track
import suspension

def p(name, wrapper=lambda x: x):
    """ Just a helper to print global variable easily """
    print(name, wrapper(globals()[name]))

def trace(value, name=None):
    if name is None:
        print(value)
    else:
        print(name + ":", value)
    return value

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

print("Optimizing kinematics - step 1 ...")
with warnings.catch_warnings():
    warnings.simplefilter("error")
    #result = scipy.optimize.minimize(objective,
    #                                 initial,
    #                                 method="SLSQP",
    #                                 options={"maxiter": 5000},
    #                                 constraints=[{"type": "eq", "fun": fun} for fun in equalities] +
    #                                             [{"type": "ineq", "fun": fun} for fun in inequalities])
    result = scipy.optimize.minimize(objective,
                                     initial,
                                     method="COBYLA",
                                     options={"maxiter": 50000,
                                              "rhobeg": 1,
                                              "catol": 1},
                                     constraints=[{"type": "ineq", "fun": fun} for fun in equalities] +
                                                 [{"type": "ineq", "fun": lambda p, fun=fun: -fun(p)} for fun in equalities] +
                                                 [{"type": "ineq", "fun": fun} for fun in inequalities])
    print(result)
params = result.x
print("done")

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import numpy


    print(params)
    width = suspension_width(params)
    height = suspension_height(params)

    print("width, height:", width, height)
    print("up travel:", (math.sin(params[ARM_UP_ANGLE]) - math.sin(params[ARM_NEUTRAL_ANGLE])) * params[ARM_LENGTH])
    print("down travel:", (math.sin(params[ARM_NEUTRAL_ANGLE]) - math.sin(params[ARM_DOWN_ANGLE])) * params[ARM_LENGTH])

    plot_wheel_forces(params)

    angles = numpy.linspace(params[ARM_DOWN_ANGLE], params[ARM_UP_ANGLE], 200)
    heights = (numpy.sin(angles) - numpy.sin(params[ARM_NEUTRAL_ANGLE])) * params[ARM_LENGTH]
    forces = numpy.vectorize(lambda angle: wheel_force_at_angle(params, angle))(angles)

    plt.plot(heights, forces)
    plt.xlabel("suspension height")
    plt.ylabel("suspension force")
    plt.grid()
    plt.show()
