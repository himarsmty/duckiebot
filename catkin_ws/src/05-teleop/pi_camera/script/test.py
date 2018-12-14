#!/usr/bin/env python
#codin: utf-8
import rospy
import numpy as np 
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
 
cap = cv2.VideoCapture(0)
ret,frame = rap.read()
if ret:
    imshow('fd',frame)
    cv2.waitKey()