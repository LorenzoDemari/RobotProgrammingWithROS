#! /usr/bin/env python
'''
import rospy
# import ros message
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *
from robot_description.srv import *
from threading import Thread
import numpy as np

import math

import sys, select, termios, tty


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def keyboard_velocity(key, msg):
	if key == 'w':
		msg.linear.x = 0.4
		msg.angular.z = 0.0
	elif key == 's':
		msg.linear.x = -0.4
		msg.angular.z = 0.0	
	elif key == 'a':
		msg.angular.z = 0.6
		msg.linear.x = 0.0
	elif key == 'd':
		msg.angular.z = -0.6
		msg.linear.x = 0.0
	elif key == ' ':
		msg.angular.z = 0.0
		msg.linear.x = 0.0
	else:
		pass
	return msg
	
	
def main():
	rospy.init_node('teleop')
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
	client = rospy.ServiceProxy('/random_pos', RandPos)
	client2 = rospy.ServiceProxy('/random_go_to_point', RandPos2)
	rate = rospy.Rate(10)
	
	msg = Twist()
	randPos = RandPos2Request()

	while not rospy.is_shutdown():
		msg.linear.x = 0.0
		msg.linear.y = 0.0
		msg.linear.z = 0.0
		msg.angular.x = 0.0
		msg.angular.y = 0.0
		msg.angular.z = 0.0
		key = getKey()
		print(key)
		if (key == '\x03'):
			break
		elif key != 'r' and key != '\x03':
			msg = keyboard_velocity(key, msg)
			pub.publish(msg)	
		else:
			pub.publish(msg)
			try:
				rospy.wait_for_service('/random_pos')
				resp = client()
				randPos.x = resp.x
				randPos.y = resp.y
				print(randPos.x)
				rospy.wait_for_service('/random_go_to_point')
				resp = client2(randPos)		
			except rospy.ServiceException as e:
				print(e)
			print(resp)

		rate.sleep()
		
if __name__ == "__main__":
	settings = termios.tcgetattr(sys.stdin)
	main()
'''

import rospy
# import ros message
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *
from robot_description.srv import *
from threading import Thread
import numpy as np

import math

import sys, select, termios, tty


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def keyboard_velocity(key, msg):
	if key == 'w':
		msg.linear.x = 0.5
		msg.angular.z = 0.0
	elif key == 's':
		msg.linear.x = -0.5
		msg.angular.z = 0.0	
	elif key == 'a':
		msg.angular.z = 0.5
		msg.linear.x = 0.0
	elif key == 'd':
		msg.angular.z = -0.5
		msg.linear.x = 0.0
	elif key == 'q':
		msg.angular.z = 0.0
		msg.linear.x = 0.0
	else:
		pass
	return msg
	
	
def main():
	rospy.init_node('teleop')
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
	client = rospy.ServiceProxy('/random_pos', RandPos)
	client2 = rospy.ServiceProxy('/random_go_to_point', RandPos2)
	rate = rospy.Rate(10)
	
	msg = Twist()
	randPos = RandPos2Request()
	print('Press a key: \n[w] go forward \n[a] turn left \n[s] go backward \n[d] turn right \n[q] stop motion (only in theleop mode) \n[r] go to random point \n[x] go to a point in the wall (x = 0.0, y = -0.05)')
	while not rospy.is_shutdown():
		msg.linear.x = 0.0
		msg.linear.y = 0.0
		msg.linear.z = 0.0
		msg.angular.x = 0.0
		msg.angular.y = 0.0
		msg.angular.z = 0.0
		key = getKey()
		print(key)
		if key in [ 'w', 'a', 's', 'd', 'q' ]:
			msg = keyboard_velocity(key, msg)
			pub.publish(msg)	
		elif key == 'r':
			pub.publish(msg)
			try:
				rospy.wait_for_service('/random_pos')
				resp = client()
				randPos.x = resp.x
				randPos.y = resp.y
				rospy.wait_for_service('/random_go_to_point')
				resp = client2(randPos)		
			except rospy.ServiceException as e:
				print(e)
			print(resp)
		elif key == 'x':
			randPos.x = 0.
			randPos.y = -0.05
			rospy.wait_for_service('/random_go_to_point')
			resp = client2(randPos)
		elif (key == '\x03'):
			break
		else:
			print('Key without meaning')

		rate.sleep()
		
if __name__ == "__main__":
	settings = termios.tcgetattr(sys.stdin)
	main()
