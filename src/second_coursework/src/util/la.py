"""
module with various linear algebra capabilities
"""
import math
from util.decorators import *


class Quat:
    """
    Represents a quaternion
    """

    def __init__(self, w: float, x: float, y: float, z: float) -> 'Quat':
        """
        Create a `None` initialised `Quat` (all attributes are `None`)
        """
        self.w, self.x, self.y, self.z = w, x, y, z

    @derive
    def __deconstruct__(self): pass

    @derive
    def __repr__(self): pass

    @derive
    def clone(self) -> 'Quat': pass

    @classmethod
    def from_euler(cls: 'Quat', roll: float = 0, pitch: float = 0, yaw: float = 0) -> 'Quat':
        """
        Create a `Quat` representing the same orientation as the components of the euler
        angle provided.

        This method is taken from the wikipedia page on Euler-Quat conversions
        https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_to_quaternion_conversion
        """
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        q = cls(w, x, y, z)

        return q


class Vec3:
    """
    a three-tuple of `float`s, you can perform lots of fun operations on it though!
    """

    def __init__(self, x: float, y: float, z: float) -> 'Vec3':
        """
        Create a new `Vec3` from its three component parts 
        """
        self.x = x
        self.y = y
        self.z = z

    @derive
    def __deconstruct__(self): pass

    @derive
    def __repr__(self): pass

    @derive
    def clone(self) -> 'Vec3': pass

    def __iadd__(self, other: 'Vec3') -> 'Vec3':
        self.x += other.x
        self.y += other.y
        self.z += other.z
        return self

    @derive
    def __add__(self, other: 'Vec3') -> 'Vec3': pass

    def __isub__(self, other: 'Vec3') -> 'Vec3':
        self.x -= other.x
        self.y -= other.y
        self.z -= other.z
        return self

    @derive
    def __sub__(self, other: 'Vec3') -> 'Vec3': pass

    def __imul__(self, other: float) -> 'Vec3':
        self.x *= other
        self.y *= other
        self.z *= other
        return self

    @derive
    def __mul__(self, other: float) -> 'Vec3': pass

    def __itruediv__(self, other: float) -> 'Vec3':
        self.x /= other
        self.y /= other
        self.z /= other
        return self

    @derive
    def __truediv__(self, other: float) -> 'Vec3': pass

