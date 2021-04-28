#include "ros/ros.h"
#include "my_srv/ArmonicVel.h"


// let's try doing it void
bool armonicVel (my_srv::ArmonicVel::Request &req , my_srv::ArmonicVel::Response & res){
	res.vel = 0.1+2*std::sin(M_PI*req.x/7-2*M_PI/7);
	return true;
}


int main (int argc , char **argv)
{
	ros::init (argc , argv , "velocity_server");
	ros::NodeHandle n;
	ros::ServiceServer service= n.advertiseService ("/armonicvelocity",armonicVel);
	ros::spin();
	return 0;
}
