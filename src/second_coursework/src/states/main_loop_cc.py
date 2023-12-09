import smach # type: ignore
import rospy # type: ignore
from states.move_sm import MoveSM
from states.detect_sm import DetectSM 


class MainLoopCC(smach.Concurrence):
    """
    defines the core functionality of the action, runs MoveSM and DetectSM
    in parallel, terminating whenever DetectSM terminates
    """

    def __init__(self, action_server):
        def child_termination_cb(outcome_map):
            rospy.loginfo(f'MAIN_LOOP_CC.child_termination_cb({outcome_map})')
            if outcome_map['DETECT_SM'] == 'succeeded':
                return True
            return False      

        smach.Concurrence.__init__(
            self,
            outcomes=['succeeded', 'aborted'],
            default_outcome='aborted',
            input_keys=['state'],
            outcome_map={
                'succeeded': {
                    'DETECT_SM': 'succeeded',
                    'MOVE_SM': 'succeeded',
                },
            },
            child_termination_cb=child_termination_cb,
        )

        with self:
            smach.Concurrence.add(
                'MOVE_SM', 
                MoveSM(), 
                remapping={
                    'state': 'state',
                }
            )

            smach.Concurrence.add(
                'DETECT_SM',
                DetectSM(action_server),
                remapping={
                    'state': 'state',
                }
            ) 