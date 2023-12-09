import smach # type: ignore
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # type: ignore
import rospy # type: ignore
import yolov4 # type: ignore
from sensor_msgs.msg import Image # type: ignore
import cv2 # type: ignore
from cv_bridge import CvBridge # type: ignore
from geometry_msgs.msg import Point # type: ignore
from std_msgs.msg import String # type: ignore
from second_coursework.msg import SearchFeedback, SearchResult # type: ignore
import pathlib
import util.extensions
from util.mutex import Mutex
from states.move_state import MoveState
from typing import *


class ObjectDetector:
    """
    - Initialises a `yolov4.Detector` and a `rospy.Subscriber` to `/camera/image`
    - The `image_cb` is called on each frame
    """

    def __init__(self):
        self.__cv_image = None
        self.__camera_subscriber = rospy.Subscriber('/camera/image', Image, self.image_cb) 
        self.__detector = yolov4.Detector(
            gpu_id=0,
            config_path='/opt/darknet/cfg/yolov4.cfg',
            weights_path='/opt/darknet/yolov4.weights',
            lib_darknet_path='/opt/darknet/libdarknet.so',
            meta_path=(
                pathlib.Path() / 'src' / 'second_coursework' / 'config' / 'coco.data'
            ).resolve().__str__(),
        )
        self.__bridge = CvBridge()
        self.__counter = 0
        self.items: Mutex[Dict[str, int]] = Mutex({}) 


    def image_cb(self, msg: Image):
        # ignore 19 frames, only detect on the 20th
        if self.__counter > 0:
            self.__counter -= 1
            return
        else:
            self.__counter = 19

        self.__cv_image = self.__bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        resized_image = cv2.resize(
            self.__cv_image, 
            (self.__detector.network_width(), self.__detector.network_height())
        )

        new_detections = self.__detector.perform_detect(
            image_path_or_buf=resized_image,
            show_image=True,
        )
        
        # maybe some weird code to follow, but it's to do with my awful Mutex implementation
        # which should really use `with` or something. Docs are available for `Mutex.get_mut`
        def update_items(items: Dict[str, int]):
            for d in new_detections:
                if d.class_name in items.keys():
                    items[d.class_name] += 1
                else:
                    items[d.class_name] = 1

        self.items.get_mut(update_items)


    def get_feedback(self) -> SearchFeedback:
        """
        returns a `SearchFeedback` message representing the objects found so far
        and their counts
        """
        def convert_to_feedback(d: Dict[str, int]) -> SearchFeedback:
            items = []
            counts = []
            for k, v in d.items():
                items.append(String(data=k))
                counts.append(v)

            return SearchFeedback(
                items=items,
                item_counts=counts,
            )

        return self.items.get_mut(convert_to_feedback)


class DetectState(smach.State):
    """
    Looping state that calls itself at a rate of 5Hz, terminates once a cake
    has been detected (uses `ObjectDetector`)
    """

    def __init__(self, action_server):
        smach.State.__init__(
            self,
            outcomes=['continue', 'found_cake'],
            input_keys=['state'],
        )
        self.__object_detector = None
        self.__action_server = action_server

    def execute(self, _) -> str:
        if not self.__object_detector: 
            self.__object_detector = ObjectDetector()

        # publish feedback at a rate of 5Hz
        rospy.Rate(5).sleep()
        feedback = self.__object_detector.get_feedback()
        self.__action_server.publish_feedback(feedback)

        # if we found a cake, we're done :))))
        if self.__object_detector.items.get_mut(lambda items: 'cake' in items):
            result = SearchResult()
            result.items = feedback.items
            result.item_counts = feedback.item_counts
            result.stamp = rospy.Time.now()
            self.__action_server.set_succeeded(result)
            return 'found_cake'
            
        return 'continue'