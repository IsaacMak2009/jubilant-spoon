import rospy
import cv2
import numpy as np
import math
import time
from sensor_msgs.msg import LaserScan
import numba as nb

from core.camera import BaseRosCamera

NODE_NAME = "camera_node"

scan_data: LaserScan = LaserScan()


@nb.njit
def rayCasting(image, point, angle_range=(-180, 0), num_rays=180):
    height, width = image.shape[:2]
    rays = np.linspace(angle_range[0], angle_range[1], num_rays)

    result = []
    distance = []
    point = np.array(point)
    for angle in rays:
        theta = np.radians(angle)
        direction = np.array([np.cos(theta), np.sin(theta)])

        for step in range(900):
            x, y = map(int, point + direction * step)
            if image.shape[1] > x >= 0 and 0 <= y < image.shape[0]:
                if image[y, x]:
                    result.append((int(x), int(y)))
                    distance.append(step)
                    break
            elif not (image.shape[1] > x >= 0 or y <= 0):
                result.append((int(x), int(y)))
                distance.append(900)
                break
        return result, distance


def convertLidarToLaserScan(lidar_data, offset=0):
    global scan_data

    scan_msg = LaserScan()
    new_data = [scan_data.ranges[i] for i in range(len(scan_data.ranges))]
    leng = len(new_data)
    k = 360 / leng
    for i in range(leng):
        angle = (int(i * k) + 360 + offset) % 360
        if angle <= len(lidar_data) and lidar_data[angle] != 900:
            new_data[i] = lidar_data[angle] * math.sqrt(3) / 10000

    # Set the necessary parameters
    scan_msg.header.stamp = rospy.Time.now()
    scan_msg.header.frame_id = scan_data.header.frame_id
    scan_msg.angle_min = scan_data.angle_min
    scan_msg.angle_max = scan_data.angle_max
    scan_msg.angle_increment = scan_data.angle_increment
    scan_msg.time_increment = scan_data.time_increment
    scan_msg.range_min = scan_data.range_min
    scan_msg.range_max = scan_data.range_max

    # Fill in the range data
    scan_msg.ranges = new_data
    scan_msg.intensities = scan_data.intensities

    return scan_msg


def scan_callback(msg):
    global scan_data
    scan_data = msg


def main():
    rospy.init_node(NODE_NAME)
    rospy.loginfo(f"{NODE_NAME} started!")
    cam = BaseRosCamera("Depth1", "/camera/depth/image_raw", "passthrough")
    rospy.loginfo("Camera is ready!")

    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

    scan_sub = rospy.Subscriber('/my_scan', LaserScan, scan_callback)
    rospy.loginfo("waiting...")
    try:
        rospy.wait_for_message("/my_scan", LaserScan, timeout=5)
        use_lidar = True
    except rospy.ROSException:
        rospy.logwarn("No lidar")
        use_lidar = False
    rospy.sleep(1)

    frame_cnt = 0
    t = time.time()
    print(np.max(cam.read()))
    bg_noise = np.loadtxt("depth.txt")
    print(bg_noise)
    print(cam.read())

    max_diff = 25
    while not rospy.is_shutdown():
        frame_cnt += 1
        rospy.Rate(10).sleep()
        frame = cam.read()
        diff = np.zeros_like(frame)
        diff = diff.astype(np.uint8)
        err = np.abs(frame - bg_noise)
        diff[err > max_diff] = 255
        diff[frame == 0 | frame > 1050] = 0
        diff = diff[:, ::-1]
        if frame_cnt == 5:
            print(diff)
        diff = cv2.medianBlur(diff, 7)
        diff = cv2.medianBlur(diff, 3)

        rgb = cv2.cvtColor(diff, cv2.COLOR_GRAY2RGB)
        x, y = 320, 540

        result, distance = rayCasting(diff, (x, y))
        for i in range(len(result) - 1):
            x2, y2 = result[i]
            if distance[i] < 900:
                cv2.line(rgb, (x, y), (x2, y2), (0, 0, 255), 1)
            else:
                cv2.line(rgb, (x, y), (x2, y2), (255, 0, 0), 1)

        scan_msg = convertLidarToLaserScan(distance, offset=90)
        pub.publish(scan_msg)

        cv2.imshow("image", rgb)
        cv2.imshow("diff", diff)
        kc = cv2.waitKey(1)

        if kc == 27:
            break

        if frame_cnt == 50:
            rospy.loginfo(f"{frame_cnt} frames in {t - time.time():.3f}, FPS: {frame_cnt / (t - time.time()):.3f}")
            frame_cnt = 0
            t = time.time()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
