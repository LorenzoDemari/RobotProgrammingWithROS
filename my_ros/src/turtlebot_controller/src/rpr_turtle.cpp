#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include "turtlebot_controller/Vel.h"
#include "my_srv/Velocity.h"


ros::Publisher pub;
ros::Publisher pub2;
ros::ServiceClient client3;
int count = 10;


void turtleCallback(const turtlesim::Pose::ConstPtr& msg)
{
	ROS_INFO("Turtle subscriber@[%f, %f, %f]", msg->x, msg->y, msg->theta);
	geometry_msgs::Twist my_vel;
	//turtlebot_controller::Vel turtle_vel;
	
	my_srv::Velocity rec_vel;
	if (count == 10) {
	count = 0;
	rec_vel.request.min=0.0;
	rec_vel.request.max=1.0;
	client3.call(rec_vel);
	geometry_msgs::Twist vel;
	vel.linear.x = rec_vel.response.x;
	vel.angular.z = rec_vel.response.z;
	pub.publish(vel);
	turtlebot_controller::Vel mymsg;
	mymsg.name = "linear";
	mymsg.vel = vel.linear.x;
	pub2.publish(mymsg);
	}
	count++;
	
	/*
	if (msg->x > 9.0){
		my_vel.linear.x = 1.0;
		my_vel.angular.z = 1.0;
	}
	else if (msg->x < 2.0){
		my_vel.linear.x = 1.0;
		my_vel.angular.z = -1.0;
	}
	else{
		my_vel.linear.x = 1.0;
		my_vel.angular.z = 0.0;
	}*/
	//turtle_vel.name = "linear";
	//turtle_vel.vel = my_vel.linear.x;
	//pub.publish(my_vel);
	//pub2.publish(turtle_vel);
}



int main (int argc, char **argv)
{
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS
	//system
	ros::init(argc, argv, "rpr_turtle");
	ros::NodeHandle nh;
	// Define the service client
	ros::ServiceClient client0 = nh.serviceClient<turtlesim::Kill>("/kill");
	turtlesim::Kill srv0;
	srv0.request.name = "turtle1";
	client0.call(srv0);
	ros::ServiceClient client1 = nh.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn srv1;
	srv1.request.x = 2.0;
	srv1.request.y = 1.0;
	srv1.request.theta = 0.0;
	srv1.request.name = "rpr_turtle";
	client1.call(srv1);
	client3 = nh.serviceClient<my_srv::Velocity>("/velocity");
	// Define the publisher to turtle's posi@on
	pub = nh.advertise<geometry_msgs::Twist> ("rpr_turtle/cmd_vel", 1);
	pub2 = nh.advertise<turtlebot_controller::Vel> ("rpr_turtle/turtle_vel", 1);
	// Define the subscriber to turtle's posi@on
	ros::Subscriber sub = nh.subscribe("rpr_turtle/pose", 1,turtleCallback);
	ros::spin();
	return 0;
}
