#!/usr/bin/env python
#coding: utf-8
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import PIL.Image as pImage
import sys
from duckietown_msgs.msg import Twist2DStamped
import time


def callback(data):
    line_state = 1  #寻迹状态
    label_state = 2  #识别状态
    turn_state = 3  #转向状态
    stop_state = 4  #停止状态
    # cmd = forward left right stop
    vel_puber = rospy.Publisher(
        "/duckiebot1/joy_mapper_node/car_cmd", Twist2DStamped, queue_size=1)
    car_cmd_msg = Twist2DStamped()
    car_cmd_msg.header.seq = 0
    now = rospy.get_rostime()
    car_cmd_msg.header.stamp = now

    # print('received an image')
    global bridge, now_state
    # print('now state :%d'%(now_state))
    #转为rgb图
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    #求宽、长、通道数
    [rows, cols, channels] = frame.shape
    #黄色背景图 有助于识别红色标志
    yello_background = np.zeros([rows, cols, channels], np.uint8)
    yello_background[:, :, 1] = np.zeros([rows, cols]) + 255
    yello_background[:, :, 2] = np.zeros([rows, cols]) + 255
    yello_frame = cv2.addWeighted(frame, 0.9, yello_background, 0.1, 0)
    hsv = cv2.cvtColor(yello_frame, cv2.COLOR_BGR2HSV)
    #定义红色无图的HSV阈值
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    #对图片进行二值化处理
    mask = cv2.inRange(hsv, lower_red, upper_red)
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    cv2.line(mask1, ((cols - 1) * 1 / 2, 0), ((cols - 1) * 1 / 2, rows - 1),
             150)
    cv2.line(mask1, ((cols - 1) * 1 / 2 + 70, 0),
             ((cols - 1) * 1 / 2 + 70, rows - 1), 150)
    cv2.line(mask1, ((cols - 1) * 1 / 2 - 70, 0),
             ((cols - 1) * 1 / 2 - 70, rows - 1), 150)
    cv2.line(yello_frame, (0, (rows - 1) * 4 / 5),
             ((cols - 1), (rows - 1) * 4 / 5), [255, 0, 0])

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 150, 200)
    ##循左右路
    for i in range(0, (cols - 1) * 1 / 2, 1):
        left_point_col = i
        if (canny[(rows - 1) * 7 / 9][i] == 255):
            break
    for i in range(cols - 1, (cols - 1) * 1 / 2, -1):
        right_point_col = i
        if (canny[(rows - 1) * 7 / 9][i] == 255):
            break
    ##########循线状态##########
    global last_state
    if (now_state == line_state):
     
        #确定标志范围
        label_left_up_row = rows
        label_right_down_row = 0
        label_left_up_col = cols
        label_right_down_col = 0
        label_squre = 0
        #标志左右半部分
        left_squre = right_squre = 0
        global label_name
        for row in range(0, rows, 1):
            for col in range((cols - 1) / 2 - 70, (cols - 1) / 2 + 70, 1):
                if (mask[row][col] == 255):
                    if (label_left_up_row > row):
                        label_left_up_row = row
                    if (label_left_up_col > col):
                        label_left_up_col = col
                    if (label_right_down_row < row):
                        label_right_down_row = row
                    if (label_right_down_col < col):
                        label_right_down_col = col
                    label_squre += 1
        print(label_squre)
        delta = (left_point_col + right_point_col) / 2 - cols / 2
        delta = -delta
        # cv2.circle(canny, (left_point_col, (rows - 1) / 9 * 7), 5, 150, -1)
        # cv2.circle(canny, (right_point_col, (rows - 1) / 9 * 7), 5, 150, -1)
        # cv2.imshow('canny', canny)
        print('循线状态校准：')
        if(delta > 0):
            print('向左 %d'%delta)
        else:
            print('向右 %d'%delta)
        if (abs(delta) > 20):  #校准
            car_cmd_msg.v = 0.0
            car_cmd_msg.omega = delta * 0.04
            vel_puber.publish(car_cmd_msg)
            if(delta > 0):
                time.sleep(0.15)
            else:
                time.sleep(0.16)
            car_cmd_msg.v = 0.0
            car_cmd_msg.omega = 0.0
            vel_puber.publish(car_cmd_msg)
            time.sleep(0.1)
        if (abs(delta) <= 20):  #前进
            car_cmd_msg.v = 0.2
            car_cmd_msg.omega = 0.0
            vel_puber.publish(car_cmd_msg)
            time.sleep(0.2)
            car_cmd_msg.v = 0.0
            car_cmd_msg.omega = 0.0
            vel_puber.publish(car_cmd_msg)
            time.sleep(0.1)
        if (label_squre > 1000 and label_squre < 7500):  #转换判断方式
            car_cmd_msg.v = 0.0
            car_cmd_msg.omega = 0.0
            vel_puber.publish(car_cmd_msg)
            time.sleep(1)
            now_state = label_state
            if(last_state == turn_state):
                print('last state is turn state')
                car_cmd_msg.v = 0.2
                car_cmd_msg.omega = 0.0
                vel_puber.publish(car_cmd_msg)
                time.sleep(0.2)
                car_cmd_msg.v = 0.0
                car_cmd_msg.omega = 0.0
                vel_puber.publish(car_cmd_msg)
                time.sleep(0.1)      
                last_state = 5
            
    ##########精确状态##########
    if (now_state == label_state):
        print('LLLLLabel')
        #确定标志范围
        label_left_up_row = rows
        label_right_down_row = 0
        label_left_up_col = cols
        label_right_down_col = 0
        label_squre = 0
        #标志左右半部分
        left_squre = right_squre = 0
        global label_name
        for row in range(0, rows, 1):
            for col in range((cols - 1) / 2 - 70, (cols - 1) / 2 + 70, 1):
                if (mask[row][col] == 255):
                    if (label_left_up_row > row):
                        label_left_up_row = row
                    if (label_left_up_col > col):
                        label_left_up_col = col
                    if (label_right_down_row < row):
                        label_right_down_row = row
                    if (label_right_down_col < col):
                        label_right_down_col = col
                    label_squre += 1
        cv2.rectangle(mask1, (label_left_up_col, label_left_up_row),
                      (label_right_down_col, label_right_down_row), 255, 1)
        #左边像素数
        for col in range(label_left_up_col,
                         (label_right_down_col + label_left_up_col) / 2, 1):
            if (mask[label_left_up_row + 3][col] == 255):
                left_squre += 1
        #右边像素数
        for col in range((label_right_down_col + label_left_up_col) / 2,
                         label_right_down_col, 1):
            if (mask[label_left_up_row + 3][col] == 255):
                right_squre += 1
        print('left squre right squre')
        print((left_squre,right_squre))
        if (1000 < label_squre < 3000):
            if (left_squre > 5 and right_squre < 5):  #左转标志
                label_name = 'left label'
            elif (left_squre < 5 and right_squre > 5):  #右转标志
                label_name = 'right label'
            elif (left_squre >= 5 and right_squre >= 5):  #停止标志
                label_name = 'stop label'
            print(label_name)
        #标志中心
        labelcenter_col = (label_left_up_col + label_right_down_col)/2
        labelcenter_col_delta = labelcenter_col - cols/2
        print(labelcenter_col_delta)
        # print()
        print('label squre %d' % label_squre)
        if (abs(labelcenter_col_delta) > 20):  #校准
            car_cmd_msg.v = 0.0
            car_cmd_msg.omega = labelcenter_col_delta * -0.04
            vel_puber.publish(car_cmd_msg)
            time.sleep(0.1)
            car_cmd_msg.v = 0.0
            car_cmd_msg.omega = 0.0
            vel_puber.publish(car_cmd_msg)
            time.sleep(0.1)
        ##继续走进
        elif (label_squre < 8500):
            car_cmd_msg.v = 0.2
            car_cmd_msg.omega = 0.0
            vel_puber.publish(car_cmd_msg)
            time.sleep(0.2)
            car_cmd_msg.v = 0.0
            car_cmd_msg.omega = 0.0
            vel_puber.publish(car_cmd_msg)
            time.sleep(0.1)
        else:
            car_cmd_msg.v = 0.0
            car_cmd_msg.omega = 0.0
            vel_puber.publish(car_cmd_msg)
            time.sleep(0.2)
            now_state = turn_state

    # #########判断方向状态############
    if (now_state == turn_state):
        print('TTTURN')
        car_cmd_msg.v = 0.0
        car_cmd_msg.omega = 0.0
        vel_puber.publish(car_cmd_msg)
        time.sleep(0.2)
        if (label_name == 'left label'):  #左转标志
            print('@@turn state: left label')
            car_cmd_msg.v = 0.0
            car_cmd_msg.omega = 1.5
            vel_puber.publish(car_cmd_msg)
            time.sleep(1.1)
            car_cmd_msg.v = 0.0
            car_cmd_msg.omega = 0.0
            vel_puber.publish(car_cmd_msg)
            time.sleep(1)
            now_state = line_state
            last_state = turn_state
        elif (label_name == 'right label'):  #右转标志
            print('@@turn state: right label')
            car_cmd_msg.v = 0.0
            car_cmd_msg.omega = -1.5
            vel_puber.publish(car_cmd_msg)
            time.sleep(1.8)
            car_cmd_msg.v = 0.0
            car_cmd_msg.omega = 0.0
            vel_puber.publish(car_cmd_msg)
            time.sleep(1)
            now_state = line_state
            last_state = turn_state
        elif (label_name == 'stop label'):  #停止标志
            print('@@turn state: stop label')
            car_cmd_msg.v = 0.0
            car_cmd_msg.omega = 0.0
            vel_puber.publish(car_cmd_msg)
            time.sleep(3)
            now_state = stop_state
        else:
            car_cmd_msg.v = 0.0
            car_cmd_msg.omega = 0.0
            vel_puber.publish(car_cmd_msg)
            time.sleep(3)
    ##########停止状态##########
    if (now_state == stop_state):
        print('@@停止状态：stop')
        now_state = stop_state
        car_cmd_msg.v = 0.0
        car_cmd_msg.omega = 0.0
        vel_puber.publish(car_cmd_msg)
        time.sleep(30)
    cv2.imshow('frame', yello_frame)
    cv2.imshow('red_mask', mask1)
    cv2.waitKey(1)


def displayScicam():

    rospy.init_node('webcam_display', anonymous=True)
    # make a video_object and init the video object
    global bridge, now_state, last_state
    global label_name
    label_name = ''
    now_state = 1  #初始状态
    last_state = 5 #上次状态
    bridge = CvBridge()
    global last_left_dis, last_right_dis, left_dis, right_dis
    last_left_dis = left_dis = 0.0
    last_right_dis = right_dis = 400
    print('ready to receive ...')
    rospy.Subscriber('sci/image_raw', Image, callback)
    rospy.spin()


if __name__ == '__main__':
    reload(sys)
    sys.setdefaultencoding('utf-8')
    print('ready to receiving images')
    displayScicam()
