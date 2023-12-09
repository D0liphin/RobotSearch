#!/usr/bin/python3
"""
Section 2 part A:

Should create a node that implements the GetRoomCoord service. That is, a service that, given an
input string, returns a point inside the room.
"""
import rospy # type: ignore
from second_coursework.srv import GetRoomCoord, GetRoomCoordRequest, GetRoomCoordResponse # type: ignore
from geometry_msgs.msg import PoseStamped # type: ignore
from std_msgs.msg import String # type: ignore
from util.extensions import *
from util import la
from util.decorators import derive, debugrepr, rosdebug, debug
from util.types import *
from typing import *


class Room:
    """
    Contains a 4-tuple of the coordinates of a pefectly rectangular room. 
    Note that this is not quite a Plane, as it is always upright (that is, parallel to x, y)

    It allows for acquitisition of tuples that represent points in the room
    """

    class CornerInset:
        """
        inset corners to acquire the points. 
        """

        def __init__(
            self, top: float, right: float, bottom: float, left: float,
        ):
            """
            Inset the `top_<left|right>` corners by `top`, `<top|bottom>_left` corners by `left` etc.
            """
            self.top = top
            self.right = right
            self.bottom = bottom
            self.left = left

        @derive
        def __deconstruct__(self): pass

        @derive
        def __repr__(self): pass

    WaypointAcquisitionMethod = CornerInset
    """
    typing union defining all the ways we can generate points inside a room
    """

    def __init__(self, center: la.Vec3, half_width: float, half_length: float):
        """
        Initialises a `Room` from a center, a half_width and a half_height (this is 
        standard representation for lots of squares and cubes)
        """
        self.center = center
        self.half_width = half_width
        self.half_length = half_length

    @derive
    def __deconstruct__(self): pass

    @derive
    def __repr__(self) -> str: pass

    def from_edges(top: float, right: float, bottom: float, left: float, z: float = 0) -> 'Room':
        """
        Create a new room from four sides (top is the y position of the tl and tr coord)
        """
        center = la.Vec3((left + right) / 2, (top + bottom) / 2, z)
        half_width = center.x - left
        half_length = center.y - bottom
        return Room(center, half_width, half_length)

    def corners(self) -> Tuple[la.Vec3, la.Vec3, la.Vec3, la.Vec3]:
        """
        Return a tuple `Tuple[top_left, top_right, bottom_right, bottom_left]` containing the
        corners of this room
        """
        z = self.center.z
        tl = la.Vec3(
            self.center.x - self.half_width,
            self.center.y + self.half_length,
            z
        )
        br = la.Vec3(
            self.center.x + self.half_width,
            self.center.y - self.half_length,
            z
        )
        return tl, la.Vec3(br.x, tl.y, z), br, la.Vec3(tl.x, br.y, z)

    def get_waypoints(self, method: 'Room.WaypointAcquisitionMethod') -> Tuple[la.Vec3]:
        """
        Get some waypoints in the room 
        """
        if isinstance(method, Room.CornerInset):
            tl, tr, br, bl = self.corners()
            tl.y -= method.top
            tr.y -= method.top
            tr.x -= method.right
            br.x -= method.right
            br.y += method.bottom
            bl.y += method.bottom
            bl.x += method.left
            tl.x += method.left
            return tl, tr, br, bl
        else:
            raise ValueError(
                'invalid WaypointAcquisitionMethod \'{method.__class__.__name__}\''
            )


WAYPOINT_ACQUISITION_METHOD = Room.CornerInset(1, 1, 1, 1)

HOUSE_WIDTH = 13.0
HOUSE_HEIGHT = 10.7

# fmt: off
ROOM_A = Room.from_edges(10.7, 3.78, 5.97, 0   )
ROOM_B = Room.from_edges(10.7, 8.33, 5.97, 3.78)
ROOM_C = Room.from_edges(10.7, 13  , 5.97, 8.33)
ROOM_D = Room.from_edges(5.97, 3.78, 0   , 0   )
ROOM_E = Room.from_edges(5.97, 8.33, 0   , 3.78)
ROOM_F = Room.from_edges(4.71, 13  , 0   , 8.33)
# fmt: on

ROOM_WAYPOINTS: Dict[str, IndexedTuple[PoseStamped]] = {
    name: IndexedTuple(*room.get_waypoints(WAYPOINT_ACQUISITION_METHOD))
    for name, room in [
        ('A', ROOM_A),
        ('B', ROOM_B),
        ('C', ROOM_C),
        ('D', ROOM_D),
        ('E', ROOM_E),
        ('F', ROOM_F),
    ]
}


def handle_get_room_coord(request: GetRoomCoordRequest) -> GetRoomCoordResponse:
    room = request.room.data 
    point = ROOM_WAYPOINTS[room].next().get()
    return GetRoomCoordResponse(pose=PoseStamped.from_x_y_yaw(point.x, point.y, 0))


def main():
    rospy.init_node('roomservice')
    _ = rospy.Service('roomservice_service', GetRoomCoord, handle_get_room_coord)
    rospy.spin()