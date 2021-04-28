#! /usr/bin/env python

import rospy

int_var = rospy.get_param("/my_integer")
float_var = rospy.get_param("/my_float")
string_var = rospy.get_param("/my_string")
print("Int: "+str(int_var)+", Float: "+str(float_var)+" String: "+str(string_var))
