"""
Extends certain classes by adding methods to the class object.

I know that this is not the correct way of doing things, but I thought
'these are special ROS classes, let me explicitly extend them, rather
than subclassing them'. (I'm pretty sure they can't be special but, well
I wanted to be safe + I don't get any debug info anywayyy)
"""

from geometry_msgs.msg import PoseStamped # type: ignore
from std_msgs.msg import String # type: ignore
import actionlib # type: ignore
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # type: ignore
import rospy # type: ignore
from second_coursework.msg import BoundingBox
from util import la
import random
from typing import *


def scaled(self: BoundingBox, x: Tuple[float, float], y: Tuple[float, float]) -> BoundingBox:
    """
    scales in the x from x[0] to x[1] and scales in the y from y[0] to y[1]
    """
    def scale(value: float, scalar: Tuple[float, float]) -> int:
        int((value / scalar[0]) * scalar[1])

    return BoundingBox(
        width=scale(self.width, x),
        height=scale(self.height, y),
        x=scale(self.x, x),
        y=scale(self.y, y),
    )


@staticmethod
def from_pose(pose: PoseStamped) -> MoveBaseGoal:
    """
    Create a `MoveBaseGoal` from a given pose
    """
    goal = MoveBaseGoal()
    goal.target_pose = pose
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    return goal


MoveBaseGoal.from_pose = from_pose
del from_pose


@staticmethod
def from_x_y_yaw(x: float, y: float, yaw: float) -> PoseStamped:
    """
    Create a pose stamped with position `(x, y, 0)` and orientation
    `Quat.from_euler(0, 0, yaw)` 
    """
    p = PoseStamped()
    p.pose.position.x = x
    p.pose.position.y = y

    q = la.Quat.from_euler(yaw=yaw)
    p.pose.orientation.x = q.x
    p.pose.orientation.y = q.y
    p.pose.orientation.z = q.z
    p.pose.orientation.w = q.w

    return p


PoseStamped.from_x_y_yaw = from_x_y_yaw
del from_x_y_yaw


def clone(self: PoseStamped) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = self.header.frame_id
    p.header.stamp = self.header.stamp
    p.pose.position.x = self.pose.position.x
    p.pose.position.y = self.pose.position.y
    p.pose.position.z = self.pose.position.z
    p.pose.orientation.x = self.pose.orientation.x
    p.pose.orientation.y = self.pose.orientation.y
    p.pose.orientation.z = self.pose.orientation.z
    p.pose.orientation.w = self.pose.orientation.w
    return p


PoseStamped.clone = clone
del clone


def get_random_nearby_point(self: PoseStamped, radius: float) -> PoseStamped:
    """
    Returns a random nearby point within a given radius.
    TODO: use this as a backup 
    """
    def random_offset():
        r = random.random()
        r = r * 2 - 1 # -1..1
        return r * radius

    p = self.clone()
    p.pose.position.x += random_offset()
    p.pose.position.y += random_offset()
    return p


PoseStamped.get_random_nearby_point = get_random_nearby_point
del get_random_nearby_point


@staticmethod
def from_str(data: str) -> String:
    """
    build a string with some data already in it (specified by `data`)
    """
    return String(data=data)
String.from_str = from_str
del from_str


