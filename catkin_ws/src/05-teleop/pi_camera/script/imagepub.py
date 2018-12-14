#!/usr/bin/env python
# coding: utf-8
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import sys


def SciImagePub():
    rospy.init_node('cameraprocess', anonymous=True)
    img_pub = rospy.Publisher('sci/image_raw', Image, queue_size=2)
    rate = rospy.Rate(5)
    cap = cv2.VideoCapture(0)
    scaling_factor = 0.5
    bridge = CvBridge()
    if not cap.isOpened():
        sys.stdout.write('sci camera is not open!\n')
        return -1
    count = 0
    print('camera opened sucessfully')
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            count += 1
        else:
            rospy.loginfo('capturing image failed.')
        if count == 3:
            count = 0
            frame = cv2.resize(
                frame,
                None,
                fx=scaling_factor,
                fy=scaling_factor,
                interpolation=cv2.INTER_AREA)
            msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_pub.publish(msg)
        #print ('publishing frame ...')
        rate.sleep()


if __name__ == '__main__':
    SciImagePub()
