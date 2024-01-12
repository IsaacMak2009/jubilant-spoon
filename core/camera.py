import rospy
import cv2
import numpy as np
from loguru import logger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class BaseRosCamera:
    def __init__(self, name: str, topic_name: str="/camera/color/image_raw", encoding: str="bgr8"):
        '''
        :param name: string, name of camera
        :param topic_name: string, image topic such as /camera/rgb/image_raw
        :param encoding: string, image encoding bgr8 for rgb, passthrough for depth
        '''
        self.frame: np.array = np.array(-1)
        self.frame_cnt = 0
        self.isready = False
        self.name = name
        self.encoding = encoding
        self.topic_name = topic_name
        self.subscriber = rospy.Subscriber(topic_name, Image, self.callback)
        self.bridge = CvBridge()

        logger.info(f"[@camera {self.name}] Initializing camera...")

        image = rospy.wait_for_message(topic_name, Image, timeout=5)
        self.callback(image)

        logger.debug(f"[@camera {self.name}] Topic: " + topic_name)
        logger.debug(f"[@camera {self.name}] Encoding: " + encoding)
        logger.debug(f"[@camera {self.name}] Image Data: " + str(self.frame.shape))

        self.isready = True
        logger.success("Camera is ready!")

    def callback(self, data):
        self.frame = self.bridge.imgmsg_to_cv2(data, desired_encoding=self.encoding)
        self.frame_cnt += 1

    def read(self) -> np.array:
        '''
        return the data from camera sensor
        '''
        return self.frame

    def safe_read(self) -> np.array:
        '''
        like .read() module, safer
        '''
        return self.frame.copy()
