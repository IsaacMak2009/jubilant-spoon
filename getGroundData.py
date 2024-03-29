import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

from core.camera import BaseRosCamera

if __name__ == '__main__':
    rospy.init_node("test")
    cam = BaseRosCamera("Depth1", "/camera/depth/image_raw", "passthrough")
    final = np.zeros_like(cam.read()).astype(int)

    for i in range(4):
        depth = cam.read().astype(int)
        idx = final==0
        final[idx] = depth[idx]
        time.sleep(0.1)
        print("Getting img #"+str(i))


    depth = final
    print("saving...")
    np.savetxt("depth.txt", depth.astype(int), fmt="%d")