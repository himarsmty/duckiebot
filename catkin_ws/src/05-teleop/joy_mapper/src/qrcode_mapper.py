#!/usr/bin/env python
#coding:utf-8
import rospy
import math
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from std_msgs.msg import String

from __builtin__ import True


def callback(msg):
    car_cmd_msg = Twist2DStamped()
    car_cmd_msg.header.seq = 0
    now = rospy.get_rostime()
    car_cmd_msg.header.stamp = now
    if msg.data == '前进':
        print(msg.data)
        car_cmd_msg.v = 0.1
        car_cmd_msg.omega = 0.0
    if msg.data == '右转':
        print(msg.data)
        car_cmd_msg.v = 0.0
        car_cmd_msg.omega = -1.0
    if msg.data == '左转':
        print(msg.data)
        car_cmd_msg.v = 0.0
        car_cmd_msg.omega = 1.0
    pub_car_cmd.publish(car_cmd_msg)


class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " % (self.node_name))

        self.joy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()

        # Setup Parameters
        self.v_gain = self.setupParam("~speed_gain", 0.41)
        self.omega_gain = self.setupParam("~steer_gain", 8.3)
        self.bicycle_kinematics = self.setupParam("~bicycle_kinematics", 0)
        self.steer_angle_gain = self.setupParam("~steer_angle_gain", 1)
        self.simulated_vehicle_length = self.setupParam(
            "~simulated_vehicle_length", 0.18)

        # Publications
        self.pub_joy_override = rospy.Publisher(
            "~joystick_override", BoolStamped, queue_size=1)
        self.pub_parallel_autonomy = rospy.Publisher(
            "~parallel_autonomy", BoolStamped, queue_size=1)
        self.pub_anti_instagram = rospy.Publisher(
            "anti_instagram_node/click", BoolStamped, queue_size=1)
        self.pub_e_stop = rospy.Publisher(
            "wheels_driver_node/emergency_stop", BoolStamped, queue_size=1)
        self.pub_avoidance = rospy.Publisher(
            "~start_avoidance", BoolStamped, queue_size=1)

        # timer
        # self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.publishControl)
        self.param_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0), self.cbParamTimer)
        self.has_complained = False
        self.state_parallel_autonomy = False
        self.state_verbose = False

        pub_msg = BoolStamped()
        pub_msg.data = self.state_parallel_autonomy
        pub_msg.header.stamp = self.last_pub_time
        self.pub_parallel_autonomy.publish(pub_msg)

    #设置初始参数
    def cbParamTimer(self, event):
        self.v_gain = rospy.get_param("~speed_gain", 1.0)
        self.omega_gain = rospy.get_param("~steer_gain", 10)

    #设置初始参数
    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name,
                        value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == "__main__":
    rospy.init_node("qrcode_mapper", anonymous=True)
    joy_mapper = JoyMapper()
    pub_car_cmd = rospy.Publisher(
        "/duckiebot1/joy_mapper_node/car_cmd", Twist2DStamped, queue_size=1)
    rospy.Subscriber('sci/qr_cmd', String, callback=callback)
    rospy.spin()
