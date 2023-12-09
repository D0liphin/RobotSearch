"""
Defines the `PatrollingState` class, more information can be found
in the class documentation
"""

import smach_ros # type: ignore
import rospy # type: ignore
from std_msgs.msg import String # type: ignore
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # type: ignore
from second_coursework.srv import GetRoomCoord, GetRoomCoordRequest, GetRoomCoordResponse # type: ignore
import util.extensions


VALID_ROOMS = ['A', 'B', 'C', 'D', 'E', 'F']


def get_room_coord(room: str) -> GetRoomCoordResponse:
    """
    use `roomservice_service` to get a `GetRoomCoordResponse` (a.k.a a wrapper
    around a `PoseStamped` represneting a point in this room)
    """

    if room not in VALID_ROOMS:
        message = f'Service call failed: \'{room}\' is not a valid room.'
        rospy.loginfo(message)
        raise ValueError(message)
    try:
        rospy.wait_for_service('roomservice_service')
        service_proxy = rospy.ServiceProxy('roomservice_service', GetRoomCoord)
        return service_proxy(GetRoomCoordRequest(room=String(data=room)))
    except rospy.ServiceException as e:
        rospy.loginfo(f'Service call failed: {e}')


class MoveState(smach_ros.SimpleActionState):
    """
    shorthand initialisation for a more complex action state that serves 
    the purpose of going to the next point in a room
    """

    def __init__(self):
        def goal_cb(userdata, _) -> MoveBaseGoal:
            rospy.sleep(0.2)
            response = get_room_coord(userdata.state.room)
            return MoveBaseGoal.from_pose(response.pose)

        super().__init__(
            'move_base',
            MoveBaseAction,
            goal_cb=goal_cb,
            input_keys=['state'],
        )
