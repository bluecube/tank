import math

drive_speed = 4000 # mm/s
drive_sprocket_diameter = 50 # mm
motor_speed = 1000*12 # rpm

drive_sprocket_circumference = math.pi * drive_sprocket_diameter
drive_sprocket_speed = 60 * drive_speed / drive_sprocket_circumference # rpm
transmission_ratio = motor_speed / drive_sprocket_speed
print(transmission_ratio)

def p(steps):
    speed = (motor_speed / 60) * drive_sprocket_circumference
    for t1, t2 in steps:
        speed *= t1 / t2
    print(", ".join("{}:{}".format(*teeth) for teeth in steps), "-> {:.1f}m/s".format(speed/1000))

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
p(gen(3, 13, 29))
