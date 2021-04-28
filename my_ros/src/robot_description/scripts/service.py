#! /usr/bin/env python

import rospy
import random
from robot_description.srv import *

def randNumbers(req):
	x = random.uniform(-8.0, 8.0)
	y = random.uniform(-8.0, 8.0)
	res = RandPosResponse()
	res.x = x
	res.y = y
	return res

def server():
    rospy.init_node('server')
    s = rospy.Service('/random_pos', RandPos, randNumbers)
    rospy.spin()

if __name__ == "__main__":
    server()
