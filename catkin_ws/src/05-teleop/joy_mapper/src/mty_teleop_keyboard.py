#!/usr/bin/env python
import rospy
import math
import sys, select, termios, tty

from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy

from __builtin__ import True

class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.joy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()

        # Setup Parameters
        self.v_gain = self.setupParam("~speed_gain", 0.41)
        self.omega_gain = self.setupParam("~steer_gain", 8.3)
        self.bicycle_kinematics = self.setupParam("~bicycle_kinematics", 0)
        self.steer_angle_gain = self.setupParam("~steer_angle_gain", 1)
        self.simulated_vehicle_length = self.setupParam("~simulated_vehicle_length", 0.18)

        # Publications
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_joy_override = rospy.Publisher("~joystick_override", BoolStamped, queue_size=1)
        self.pub_parallel_autonomy = rospy.Publisher("~parallel_autonomy",BoolStamped, queue_size=1)
        self.pub_anti_instagram = rospy.Publisher("anti_instagram_node/click",BoolStamped, queue_size=1)
        self.pub_e_stop = rospy.Publisher("wheels_driver_node/emergency_stop",BoolStamped,queue_size=1)
        self.pub_avoidance = rospy.Publisher("~start_avoidance",BoolStamped,queue_size=1)

        # timer
        # self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.publishControl)
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)
        self.has_complained = False
        self.state_parallel_autonomy = False
        self.state_verbose = False

        pub_msg = BoolStamped()
        pub_msg.data = self.state_parallel_autonomy
        pub_msg.header.stamp = self.last_pub_time
        self.pub_parallel_autonomy.publish(pub_msg)

    #设置初始参数 
    def cbParamTimer(self,event):
        self.v_gain = rospy.get_param("~speed_gain", 1.0)
        self.omega_gain = rospy.get_param("~steer_gain", 10)
    #设置初始参数
    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def publishControl(self):
        settings = termios.tcgetattr(sys.stdin)
        while(1):
            key = self.getKey(settings)
            car_cmd_msg = Twist2DStamped()
            car_cmd_msg.header.stamp = self.joy.header.stamp

            if key == 'w':
                car_cmd_msg.v = 0.1
                car_cmd_msg.omega = 0.0
            elif key == 's':
                car_cmd_msg.v = -0.1
                car_cmd_msg.omega = 0.0               
            elif key == 'a':
                car_cmd_msg.v = 0.0
                car_cmd_msg.omega = 1.0
            elif key == 'd':
                car_cmd_msg.v = 0.0
                car_cmd_msg.omega = -1.0
            elif (key == '\x03'):
                break
            else:
                car_cmd_msg.v = 0.0
                car_cmd_msg.omega = 0.0           
            if self.bicycle_kinematics:
                # Implements Bicycle Kinematics - Nonholonomic Kinematics
             # see https://inst.eecs.berkeley.edu/~ee192/sp13/pdf/steer-control.pdf
                steering_angle = self.joy.axes[3] * self.steer_angle_gain
                car_cmd_msg.omega = car_cmd_msg.v / self.simulated_vehicle_length * math.tan(steering_angle)
            else:
                # Holonomic Kinematics for Normal Driving
                car_cmd_msg.omega = self.joy.axes[3] * self.omega_gain
            self.pub_car_cmd.publish(car_cmd_msg)
    #获取键盘输入
    def getKey(self,settings):
	    tty.setraw(sys.stdin.fileno())
	    select.select([sys.stdin], [], [], 0)
	    key = sys.stdin.read(1)
	    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	    return key


if __name__ == "__main__":
    rospy.init_node("my_teleop_keyboard",anonymous=False)
    joy_mapper = JoyMapper()
    joy_mapper.publishControl()
    rospy.spin()
