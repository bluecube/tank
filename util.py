import math

from codecad.util import Vector

class Approx:
    def __init__(self, value, relative=1e-12, absolute=1e-12):
        self.value = value
        self.allowance = max(abs(value) * relative, absolute)

    def __eq__(self, other):
        return abs(self.value - other) < self.allowance

    def __str__(self):
        return "{} +- {}".format(self.value, self.allowance)

def perpendicular(v):
    """ Perpendicular vector to v (2D only) """
    v = v.flattened()
    return Vector(v.y,  -v.x)

def point_to_line_distance(p, l1, l2):
    """ Calculate signed perpendicular distance from p to l1-l2 (2D vectors only) """
    p = p.flattened()
    l1 = l1.flattened()
    l2 = l2.flattened()

    return perpendicular((l2 - l1).normalized()).dot(p - l1)

def outer_tangent(p1, r1, p2, r2):
    """ Return a pair of points on circles (p1, r1) and (p2, r2) that form
    a line tangent to both of these circles.
    This is always an outer tangent and which one of the two outer tangents is returned
    is determined by the order of the parameters (it's the one to the left from
    line segment p1, p2). """

    print("inputs", p1, r1, p2, r2)

    dp = p2 - p1
    dr = r2 - r1
    dp2 = dp.abs_squared()
    tmp = Vector(math.sqrt(dp2 - dr*dr), dr)
    direction = Vector(perpendicular(dp).dot(tmp), -dp.dot(tmp)) / dp2

    ret1 = p1 + direction * r1
    ret2 = p2 + direction * r2

    dret = ret2 - ret1
    assert abs(ret1 - p1) == Approx(r1)
    assert abs(ret2 - p2) == Approx(r2)
    assert dret.dot(ret1 - p1) == Approx(0)
    assert dret.dot(ret2 - p2) == Approx(0)

    return (ret1, ret2)
