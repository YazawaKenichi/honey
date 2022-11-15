#/usr/bin/env python3
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import os

i = 0

def callback(data):
    global i
    print(i)
    try:
        bridge = CvBridge()
        cv2_array = bridge.imgmsg_to_cv2(data)
        cv2.imwrite("../images/" + str(i).zfill(6) + ".png", cv2_array)
        # rospy.loginfo(cv2_array)
    except Exception as err:
        rospy.logerr(err)
    if i >= 300:
        i = 0
    else:
        i = i + 1

def listener():
    rospy.init_node('hogehoge_listener', anonymous = True)
    rospy.Subscriber("/camera/fake_cam_color/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    if not os.path.exists("../images/"):
        os.mkdir('../images/')
    listener()

