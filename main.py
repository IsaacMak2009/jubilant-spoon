import cv2
import rospy
from core.camera import BaseRosCamera

def main():
    rospy.init_node("RosCode", anonymous=True)
    camera = BaseRosCamera("MyCam")

    while not rospy.is_shutdown():
        frame = camera.read()
        cv2.imshow("Hello", frame)
        if cv2.waitKey(1) in [27, ord('q')]:
            break
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()