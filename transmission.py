import math

import codecad
from codecad.shapes import *

import parameters
import track

drive_sprocket_tooth_count = 11

motor_max_rpm = 930 * 11.6 # rpm, motor Kv * motor voltage
target_max_speed = 7000 # mm/s, top speed at no load

drive_sprocket_angle = 2 * math.pi / drive_sprocket_tooth_count
drive_sprocket_working_diameter = 0.5 * track.segment_length / math.sin(drive_sprocket_angle / 2) + \
                                  track.connector_length / math.sin(drive_sprocket_angle)
drive_sprocket_circumference = (track.segment_length + track.connector_length) * drive_sprocket_tooth_count
drive_sprocket_max_rpm = 60 * target_max_speed / drive_sprocket_circumference # rpm
target_transmission_ratio = motor_max_rpm / drive_sprocket_max_rpm
print("drive sprocket working diameter", drive_sprocket_working_diameter)
print("target transmission ratio", target_transmission_ratio)

def p(steps):
    ratio = 1
    for t1, t2 in steps:
        ratio *= t2 / t1

    speed = (motor_max_rpm / 60) * drive_sprocket_circumference / ratio
    print(", ".join("{}:{}".format(*teeth) for teeth in steps), "({:.1f}:1) -> {:.1f}m/s".format(ratio, speed/1000))

def gen(steps, t1, t2max):
    l = []
    t2 = t2max
    for i in range(steps):
        while math.gcd(t1, t2) != 1:
            t2 -= 1
        l.append((t1, t2))
        t2 -= 3

    l.reverse()
    return l

#for steps in range(2, 5):
#    for t1 in range(10, 16):
#        t2 = round(((motor_speed / 60) * t1**steps * drive_sprocket_circumference / drive_speed)**(1/steps))
#        #t2 = round(t1 * (transmission_ratio**(1/steps)))
#
#        if math.gcd(t1, t2) != 1:
#            continue
#
#        p([(t1, t2)] * steps)

print()
#p([(12, 33)] * 2)
#p([(13, 25)] * 3)
p(gen(3, 13, 24))
