"""
Defines the main State machine logic for the action server
"""

import smach # type: ignore
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # type: ignore
import util.extensions
from util import la
import rospy # type: ignore
from states.move_state import MoveState
from states.main_loop_cc import MainLoopCC


class MainSM(smach.StateMachine):
    """
    The main StateMachine, looks like this

    ```plaintext
    
    ```
    """

    def __init__(self, action_server): 
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted'])

        with self:
            smach.StateMachine.add(
                'GO_TO_ROOM',
                MoveState(),
                transitions={
                    'succeeded': 'MAIN_LOOP',
                    'preempted': 'GO_TO_ROOM',
                    'aborted': 'GO_TO_ROOM',
                },
                remapping = {
                    'state': 'state',
                }
            )        

            smach.StateMachine.add(
                'MAIN_LOOP', 
                MainLoopCC(action_server),
                remapping={
                    'state': 'state',
                }
            )