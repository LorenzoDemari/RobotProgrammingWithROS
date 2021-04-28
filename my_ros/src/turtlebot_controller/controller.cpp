#include "ros/ros.h"
#include "turtlesim/Pose.h"

void turtleCallback(const turtlesim::Pose::ConstPtr& msg)
{
	ROS_INFO("Turtle subscriber@[%f, %f, %f]",
	msg->x, msg->y, msg->theta);
}



int main (int argc, char **argv)
{
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS
	//system
	ros::init(argc, argv, "turtlebot_subscriber");
	ros::NodeHandle nh;
	// Define the subscriber to turtle's posi@on
	ros::Subscriber sub = nh.subscribe("turtle1/pose", 1,turtleCallback);
	ros::spin();
	return 0;
}
