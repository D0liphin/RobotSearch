"""
defines the image detection part of the main loop concurrence container
"""

import smach # type: ignore
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # type: ignore
import rospy # type: ignore
import yolov4 # type: ignore
from sensor_msgs.msg import Image # type: ignore
import cv2 # type: ignore
from cv_bridge import CvBridge # type: ignore
from geometry_msgs.msg import Point # type: ignore
from std_msgs.msg import String # type: ignore
import pathlib
import util.extensions
from util.mutex import Mutex
from states.move_state import MoveState
from typing import *
from states.detect_state import DetectState 


class DetectSM(smach.StateMachine):
    def __init__(self, action_server):
        super().__init__(
            outcomes=['succeeded', 'aborted'],
            input_keys=['state'],
        )

        with self:
            smach.StateMachine.add(
                'DETECT',
                DetectState(action_server),
                remapping={
                    'state': 'state',
                },
                transitions={
                    'continue': 'DETECT',
                    'found_cake': 'succeeded',
                }
            )