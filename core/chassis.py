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
    def __init__(self, topic_name: str = "/cmd_vel", imu_topic: Optional[str] = "/imu"):
        self.imu_data: Optional[Imu] = None
        self.isready = False

        self.topic_name = topic_name
        self.imu_topic = imu_topic
        self.use_imu = True

        logger.info("Initializing Chassis")
        self.publisher = rospy.Publisher(self.topic_name, Twist, queue_size=10)

        # logger.info("Initializing Odometry...")
        # if self.odom_topic:
        #     logger.debug("Subscribing to odometry topic...")
        #     self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        #     try:
        #         rospy.wait_for_message(self.odom_topic, Odometry, timeout=3)
        #     except rospy.ROSException:
        #         logger.warning("Timeout occurred while waiting for odometry message.")
        #         self.use_odom = False
        # else:
        #     self.use_odom = False
        #
        # if not self.use_odom:
        #     logger.warning(f"odom ({self.odom_topic}) is not available")

        logger.info("Initializing Imu...")
        if self.imu_topic:
            logger.debug("Subscribing to imu topic...")
            self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)
            try:
                rospy.wait_for_message(self.imu_topic, Imu, timeout=3)
            except rospy.ROSException:
                logger.warning("Timeout occurred while waiting for imu message.")
                self.use_imu = False
        else:
            self.use_imu = False

        if not self.use_imu:
            logger.warning(f"imu ({self.imu_topic}) is not available")

        self.isready = True
        logger.success("Chassis is ready!")

    def imu_callback(self, msg: Imu):
        self.imu_data = msg

    def turn(self, deg: float):
        quaternion = (
            self.imu_data.orientation.x,
            self.imu_data.orientation.y,
            self.imu_data.orientation.z,
            self.imu_data.orientation.w,
        )

        pitch, yaw, roll = euler_from_quaternion(quaternion)
