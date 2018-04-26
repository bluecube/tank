from codecad.util import Vector

def point_to_line_distance(p, l1, l2):
    """ Calculate signed perpendicular distance from p to l1-l2 (2D vectors only) """
    p = p.flattened()
    l1 = l1.flattened()
    l2 = l2.flattened()

    u = (l2 - l1).normalized()
    v = Vector(u.y, -u.x)

    return (p - l1).dot(v)

