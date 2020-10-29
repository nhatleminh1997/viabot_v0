#!/usr/bin/env python3

#this is for differential drive scratch

import rospy
from geometry_msgs.msg import Twist

import board
import busio
import adafruit_pca9685
from adafruit_motor import motor

# system physical properties
wheel_radius = 0.0325 #in meters
wheel_base = 0.27 #in meters, distance between two wheels

# wiring: A for right motor, B for left motor
chan_PWMA = 0
chan_AIN2 = 1
chan_AIN1 = 4

chan_BIN1 = 6
chan_BIN2 = 5
chan_PWMB = 7

# initialize PCA 9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 1000

# motor A settings
pca.channels[chan_PWMA].duty_cycle = 0xffff #hold high
motorA = motor.DCMotor(pca.channels[chan_AIN1], pca.channels[chan_AIN2])

# motor B settings
pca.channels[chan_PWMB].duty_cycle = 0xffff
motorB = motor.DCMotor(pca.channels[chan_BIN2], pca.channels[chan_BIN1])

motorA.throttle = 0.0
motorB.throttle = 0.0
motor_speed_left = 0.0
motor_speed_right = 0.0


def callback(msg):
	rospy.loginfo("Received a /cmd_vel message")
	rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x,msg.linear.y, msg.linear.z))
	rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
	linear_speed = msg.linear.x
	angular_speed = msg.angular.z
	
	motor_speed_right = angular_speed*wheel_base/2 + linear_speed
	motor_speed_left = 2*linear_speed - motor_speed_right
	
	motorA.throttle = motor_speed_right
	motorB.throttle = motor_speed_left
	
def listener():
	rospy.init_node('cmd_vel_listener')
	rospy.Subscriber("/cmd_vel", Twist, callback)
	rospy.spin()
	
if __name__ == '__main__':
	listener()
	



 
