from second_coursework.msg import SearchAction, SearchGoal # type: ignore
from std_msgs.msg import String # type: ignore
import actionlib # type: ignore
import rospy # type: ignore
import sys
import util.extensions

def main():
    rospy.init_node('search_client')
    room = sys.argv[1]
    rospy.loginfo(f'searching in room \'{room}\'')

    client = actionlib.SimpleActionClient('search', SearchAction)

    rospy.loginfo(f'created client!')
    rospy.loginfo(f'waiting for server...')

    client.wait_for_server()

    rospy.loginfo(f'server found!')

    goal = SearchGoal(room=String.from_str(room))
    client.send_goal(goal)

    rospy.loginfo(f'sent goal!')
    rospy.loginfo(f'waiting for result...')

    client.wait_for_result()
    
    rospy.loginfo('done!')    
