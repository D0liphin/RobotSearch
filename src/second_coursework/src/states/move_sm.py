"""
defines the movement part of the concurrence container
"""

import smach # type: ignore
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # type: ignore
import util.extensions
from util import la
import rospy # type: ignore
from states.move_state import MoveState

class MoveSM(smach.StateMachine):
    def __init__(self):
        super().__init__(
            outcomes=['succeeded', 'aborted'],
            input_keys=['state'],
        )

        with self:
            smach.StateMachine.add(
                'MOVE',
                MoveState(),
                transitions={
                    'succeeded': 'MOVE',
                    'preempted': 'aborted',
                    'aborted': 'MOVE',
                },
            )

    