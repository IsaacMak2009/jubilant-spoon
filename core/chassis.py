import rospy
from loguru import logger
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from typing import *

class Controller:
    def __init__(self): pass
    def update(self, target: float, current: float): pass
    def reset(self): pass

class PIDController(Controller):
    def __init__(self, kp, ki, kd):
        super().__init__()
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0
        self.integral = 0
        self.previous_error = 0
        self.proportional = 0
        self.derivative = 0

    def update(self, target, current):
        self.error = target - current

        self.integral += self.ki * self.error
        self.proportional = self.kp * self.error
        self.derivative = self.kd * (self.error - self.previous_error)

        control_output = self.proportional + self.integral + self.derivative

        self.previous_error = self.error

        return control_output

    def reset(self):
        # Reset the controller's internal state
        self.error = 0
        self.integral = 0
        self.previous_error = 0


class Chassis:
    def __init__(self, topic_name: str = "/cmd_vel", odom_topic: Optional[str] = "/odom"):
        self.isready = False

        self.topic_name = topic_name
        self.odom_topic = odom_topic
        self.use_odom = False

        self.odom_data: Optional[Odometry] = None

        logger.info("Initializing Chassis")
        self.publisher = rospy.Publisher(self.topic_name, Twist, queue_size=10)

        logger.info("Initializing Odometry...")
        if self.odom_topic:
            logger.debug("Subscribing to odometry topic...")
            self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
            try:
                rospy.wait_for_message(self.odom_topic, Odometry, timeout=3)
            except rospy.ROSException:
                logger.warning("Timeout occurred while waiting for odometry message.")
                logger.warning("Please make sure odometry is available")
                self.use_odom = False
        else:
            self.use_odom = False

    def odom_callback(self, msg: Odometry):
        self.odom_data = msg

