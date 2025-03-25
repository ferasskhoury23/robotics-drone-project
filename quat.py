from Vector2 import *
import math

@dataclass(unsafe_hash=True, frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float

    # makes using these fields more memory and runtime efficient
    __slots__ = ['x', 'y', 'z', 'w']

    @staticmethod
    def from_euler_angles(roll, pitch, yaw):
        t0 = math.cos(yaw * 0.5)
        t1 = math.sin(yaw * 0.5)
        t2 = math.cos(roll * 0.5)
        t3 = math.sin(roll * 0.5)
        t4 = math.cos(pitch * 0.5)
        t5 = math.sin(pitch * 0.5)

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5
        return Quaternion(x, y, z, w)

    def __mul__(self, other: "Quaternion"):
        t, x, y, z = self.w, self.x, self.y, self.z
        a, b, c, d = other.w, other.x, other.y, other.z
        return Quaternion(w=a * t - b * x - c * y - d * z,
                          x=b * t + a * x + d * y - c * z,
                          y=c * t + a * y + b * z - d * x,
                          z=d * t + z * a + c * x - b * y)

    def conjugate(self):
        return Quaternion(-self.x, -self.y, -self.z, self.w)


def checkoverlapCircle(a: Vec2, b: Vec2, o: Vec2, radius: float):
    if o.distance(a) < radius or o.distance(b) < radius:
        return True

    # find the projection of o onto the line passing through a and b
    line = b - a
    p = line.project(o - a) + a
    if o.distance(p) > radius:
        # if the projection is outside the circle,
        # all other points would be further away from the center,
        # and therefore not inside the circle
        return False
    # since a and b are outside the circle,
    # the segment overlaps iff the segment (a,b) intersects with the ray from o to p
    #
    # if the points are on opposite sides, the angle between them would be pi
    # and 0 if they are on the same side
    return abs((b - p).angle(a - p)) > math.pi / 2

def getFoVCoverage(center: Vec2, radius: float) :
    origin = Vec2(0, 0)
    dist = origin.distance(center)
    if dist <= radius:
        return None
    return math.atan2(radius, math.sqrt(dist**2 - radius**2))
