#! /usr/bin/env python

import rospy
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf import transformations
# import ros service
from std_srvs.srv import *
from robot_description.srv import *

import math, time

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
position_ = Point()
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'wall following', 'Still']
state_ = 0
vel_ = Twist()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
msg = Twist()
# 0 - go to point
# 1 - wall following

# callbacks


def clbk_odom(msg):
    global position_, yaw_, desired_position_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]
    err_pos = math.sqrt(pow(desired_position_.y - msg.pose.pose.position.y, 2) + pow(desired_position_.x - msg.pose.pose.position.x, 2))
    if err_pos < 0.3 and vel_.linear.x == 0.0 and vel_.angular.z == 0.0:
        change_state(2)


def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }


def change_state(state):
    global state_, state_desc_, msg
    global pub
    global srv_client_wall_follower_, srv_client_go_to_point_
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
    if state_ == 2:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)
        pub.publish(msg)
        


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle
    
def go_to_random(req):
	global desired_position_
	rospy.set_param('des_pos_x', req.x)
	rospy.set_param('des_pos_y', req.y)
	print(req)
	time.sleep(2)
	desired_position_.x = rospy.get_param('des_pos_x')
	desired_position_.y = rospy.get_param('des_pos_y')
	resp = RandPos2Response()
	change_state(0)
	return resp
	
def stopped(vel):
        global vel_
        vel_ = vel
        

def main():
    time.sleep(2)
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, vel_

    rospy.init_node('bug0')

    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_vel = rospy.Subscriber('/cmd_vel', Twist, stopped)
    
    
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    srv_client_go_to_point_ = rospy.ServiceProxy(
        '/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy(
        '/wall_follower_switch', SetBool)
    srv_random_position_server = rospy.Service('/random_go_to_point', RandPos2, go_to_random)

    # initialize going to the point
    change_state(2)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue

        if state_ == 0:
            if regions_['front'] < 0.2:
                if abs(math.sqrt(desired_position_.x**2+desired_position_.y**2)-math.sqrt(position_.x**2+position_.y**2))>1.1:
                	change_state(1)
                else:
                	print('Goal not reachable\n')
                	change_state(2)

        elif state_ == 1:
            desired_yaw = math.atan2(
                desired_position_.y - position_.y, desired_position_.x - position_.x)
            err_yaw = normalize_angle(desired_yaw - yaw_)

            if regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
                change_state(0)
                
        elif state_ == 2:
                pass
		
        rate.sleep()


if __name__ == "__main__":
    main()

