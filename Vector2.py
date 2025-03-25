from dataclasses import dataclass
import math

#frozen makes the class immutable (attributes cannot be changed after creation).
#Allows the object to be used as a key in dictionaries
# or stored in sets even though it's mutable (normally not recommended, but safe here because it's frozen).
@dataclass(unsafe_hash=True, frozen=True)
class Vec2:
    x: float
    y: float

    #prevents python from creating a dict for each object , Reduces memory usage and increases performance.
    __slots__ = ['x', 'y']

    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        return Vec2(self.x * other, self.y * other)

    def __rmul__(self, other):
        return Vec2(self.x * other, self.y * other)

    def __truediv__(self, other):
        return Vec2(self.x / other, self.y / other)

    def __neg__(self):
        return Vec2(-self.x, -self.y)

    def round(self):
        return Vec2(round(self.x), round(self.y))

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def length(self):
        return math.sqrt(self.dot(self))

    def distance(self, other):
        return (other - self).length()

    def signed_area(self, other):
        """
        find the signed area of the parallelogram formed by the vectors.
        in 3D this would be the length of the cross product.
        """
        return self.x * other.y - self.y * other.x

    def angle(self, other):
        """
        find the angle from this vector to the other, in radians
        """
        return math.atan2(self.signed_area(other), self.dot(other))

    def rotate(self, angle):
        """
        rotate the vector by the angle, given in radians
        """
        cos = math.cos(angle)
        sin = math.sin(angle)
        return Vec2(self.x * cos - self.y * sin  ,  self.x * sin + self.y * cos)

    def project(self, other):
        """
        returns the projection of the other vector onto this one
        """
        # multiplication by the orthogonal projection matrix (v*v^T)/(v^T*v)
        x = self.x * self.x * other.x + self.x * self.y * other.y
        y = self.x * self.y * other.x + self.y * self.y * other.y
        squared_length = self.dot(self)
        if squared_length < 0.0001:
            # this vector is so small the projection is onto a point,
            # which is just the point itself
            return self
        return Vec2(x, y) / self.dot(self)

    def normalize(self):
        length = self.length()
        if length < 0.0001:
            # avoid dividing by zero
            return Vec2(0, 0)
        return self / length
