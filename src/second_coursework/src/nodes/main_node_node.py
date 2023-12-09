"""
Node implementing the `Search` action server. 
NOTE about formatting: Python has such a loose style guide, however, 
the ROS examples still manage to break it! I want to note that all
my code (besides for rules on max line-width) is accepted by the
pep8 style guide (or not explicitly disallowed). Additionally, I 
have had to format all code by hand for this project, for some reason
my formatter doesn't like this.
"""
import rospy     # type: ignore
import actionlib # type: ignore
import smach     # type: ignore
import smach_ros # type: ignore
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # type: ignore
from std_msgs.msg import String # type: ignore
from second_coursework.msg import SearchAction, SearchGoal, SearchFeedback # type: ignore
from second_coursework.srv import GetRoomCoord, GetRoomCoordRequest, GetRoomCoordResponse # type: ignore
import util.extensions
from util import la
import sys
from states.move_state import MoveState
from states.move_sm import MoveSM
from states.detect_sm import DetectSM
from states.main_loop_cc import MainLoopCC
from states.main_sm import MainSM
from dataclasses import dataclass


@dataclass
class SMState:
    room: str
    """The room that we should be moving around"""


@dataclass
class SMStateMut:
    pass


class SearchActionServer:
    """
    implements the server of the `Search` action, as is required by the
    section reading "The main_node should contain a class implementing
    the server of the action above."

    You can specify if the introspection server should be started as well
    by specifying `SearchActionServer(start_introspection_server=True)`
    """

    def __init__(self, start_introspection_server: bool = False):
        self.start_introspection_server = start_introspection_server

        self.__action_server = actionlib.SimpleActionServer(
            'search', 
            SearchAction, 
            execute_cb=self.execute_cb, 
            auto_start=False
        )

        self.__action_server.start()

    def execute_cb(self, goal: SearchGoal):
        state_machine = MainSM(self.__action_server)

        state_machine.userdata.state = SMState(
            room=goal.room.data,
        )

        if self.start_introspection_server:
            introspection_server = smach_ros.IntrospectionServer(
                'introspection_server', 
                state_machine, 
                '/SM_ROOT'
            )
            introspection_server.start()

        state_machine.execute()


def main():
    rospy.init_node('main_node')
    _ = SearchActionServer(start_introspection_server=False)
    rospy.spin()


