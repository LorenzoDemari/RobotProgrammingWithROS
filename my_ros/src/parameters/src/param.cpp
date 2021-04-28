#include "ros/ros.h"

int main(int argc, char **argv)
{

ros::init(argc, argv, "getting_params");

int int_var;
double double_var;
std::string string_var;

ros::param::get("/my_integer", int_var);
ros::param::get("/my_float", double_var);
ros::param::get("/my_string", string_var);

ROS_INFO("Int: %d, Float: %lf, String: %s", int_var, double_var, string_var.c_str());

}
