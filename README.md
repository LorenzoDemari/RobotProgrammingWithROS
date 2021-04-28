# RobotProgrammingWithROS
Move all the contents in the root folder.

Compile all the packages.

Create the rosbridge, because there are custom .srv files to be linked.

**Assignment 1**
- 3 terminals
- First Terminal
	- source ros.sh
	- roslaunch robot_description sim_w1.launch
- Second Terminal
	- source ros.sh
	- roslaunch robot_description my_bug0.launch
- Third Terminal
	- source ros.sh
	- rosrun robot_description my_user_interface.py
	

**Assignment 2**
- 5 terminals
- First Terminal
	- source ros.sh
	- roslaunch robot_description sim_w1.launch
- Second Terminal
	- source ros.sh
	- roslaunch robot_description my_bug0_ros2.launch
- Third Terminal
	- source ros12.sh
	- ros2 run ros1_bridge dynamic_bridge
- Fourth Terminal
	- source ros2.sh
	- ros2 run ass2_nodes service
- Fifth Terminal
	- source ros2.sh
	- ros2 run ass2_nodes teleop
