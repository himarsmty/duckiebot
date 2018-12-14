#!/usr/bin/env python
#codin: utf-8
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import zbar
import PIL.Image as pImage


def callback(data):
    # print('received an image')
    # define picture to_down' coefficient of ratio
    scaling_factor = 0.5
    global count, bridge
    count = count + 1
    if count == 1:
        count = 0
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("frame", cv_img)
        # qrcode scanner
        scanner = zbar.ImageScanner()
        scanner.parse_config('enable')
        pil = pImage.fromarray(cv_img).convert('L')
        width, height = pil.size
        raw = pil.tobytes()
        image = zbar.Image(width, height, 'Y800', raw)
        scanner.scan(image)
        cmd_puber = rospy.Publisher('sci/qr_cmd', String, queue_size = 10)   
        for symbol in image:
            cmd_data = symbol.data.decode('utf-8').encode('sjis').decode('utf-8')
            print(cmd_data)
            cmd_puber.publish(cmd_data)
        cv2.waitKey(10)
    else:
        pass


def displayScicam():

    rospy.init_node('webcam_display', anonymous=True)
    # make a video_object and init the video object
    global count, bridge
    count = 0
    bridge = CvBridge()
    print('ready to receive ...')
    rospy.Subscriber('sci/image_raw', Image, callback)
    rospy.spin()


if __name__ == '__main__':
    print('ready to receiving images')
    displayScicam()
