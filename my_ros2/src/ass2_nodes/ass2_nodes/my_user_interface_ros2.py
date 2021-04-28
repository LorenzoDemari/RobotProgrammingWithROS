# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ass2_services.srv import RandPos
from ass2_services.srv import RandPos2
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
import sys, select, termios, tty


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)

    def timer_callback(self):
        msg = Twist()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        
class Client(Node):

    def __init__(self):
        super().__init__('teleop')
        self.cli = self.create_client(RandPos, 'random_pos')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RandPos.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)

class Client2(Node):

    def __init__(self):
        super().__init__('teleop2')
        self.cli = self.create_client(RandPos2, 'random_go_to_point')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RandPos2.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        
def getKey():
    settings = termios.tcgetattr(sys.stdin)
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


def main(args=None):
	rclpy.init(args=args)
	
	pub = MinimalPublisher()
	msg = Twist()
	print('Press a key: \n[w] go forward \n[a] turn left \n[s] go backward \n[d] turn right \n[q] stop motion (only in theleop mode) \n[r] go to random point \n[x] go to a point in the wall (x = 0.0, y = -0.05)')

	while rclpy.ok():
		minimal_client = Client()
		minimal_client2 = Client2()
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
		elif key in [ 'w', 'a', 's', 'd', 'q' ]:
			msg = keyboard_velocity(key, msg)
			pub.publisher_.publish(msg)	
		elif key == 'r':
			pub.publisher_.publish(msg)
			minimal_client.send_request()
			rclpy.spin_until_future_complete(minimal_client, minimal_client.future)
			#rclpy.spin_once(minimal_client)
			if minimal_client.future.done():
				try:
					response = minimal_client.future.result()
				except Exception as e:
	                		minimal_client.get_logger().info(
	                    			'Service call failed %r' % (e,))
			else:
				minimal_client.get_logger().info(
	                    	'Result of random_position: %d , %d' %
	                    	(response.x, response.y))
	                    	
			minimal_client2.req.x = response.x
			minimal_client2.req.y = response.y
			minimal_client2.send_request()
			rclpy.spin_until_future_complete(minimal_client2, minimal_client2.future)
			#rclpy.spin_once(minimal_client2)
			if minimal_client2.future.done():
				try:
					response2 = minimal_client2.future.result()
				except Exception as e:
                			minimal_client2.get_logger().info(
                    				'Service call failed %r' % (e,))
			else:
				minimal_client2.get_logger().info(
           			'Result of random_position: %d , %d' %
             			(response.x, response.y))
		elif key == 'x':
			minimal_client2.req.x = 0.0
			minimal_client2.req.y = -0.5
			minimal_client2.send_request()
			rclpy.spin_until_future_complete(minimal_client2, minimal_client2.future)
			#rclpy.spin_once(minimal_client2)
			if minimal_client2.future.done():
				try:
					response2 = minimal_client2.future.result()
				except Exception as e:
                			minimal_client2.get_logger().info(
                    				'Service call failed %r' % (e,))
			else:
				minimal_client2.get_logger().info(
           			'Result of random_position: %d , %d' %
             			(response.x, response.y))
		else:
             		print('Key without meaning')
		minimal_client.destroy_node()
		minimal_client2.destroy_node()
            #break

    #minimal_client.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()

