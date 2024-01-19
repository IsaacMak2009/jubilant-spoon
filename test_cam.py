from core.camera import BaseRosCamera
import cv2
import rospy

if __name__ == '__main__':

    rospy.init_node("rayson_is_sb")
    cam = BaseRosCamera("color")
    cam2 = BaseRosCamera("depth", topic_name="/camera/depth/image_raw", encoding="passthrough")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
        cv2.imshow("test", cam.read())
        cv2.imshow("test2", cam2.read() / 1500)
        if cv2.waitKey(1) == ord('q'): break
    cv2.destroyAllWindows()
        