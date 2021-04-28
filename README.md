# RobotProgrammingWithROS
Move all the contents in the root folder.

Compile all the packages.

Assignment 1
- 2 terminals
- First Terminal
	- source ros.sh
	- roslaunch robot_description sim_w1.launch
- Second Terminal
	- source ros.sh
	- roslaunch robot_description my_bug0.launch
	

Assignment 2
- 5 terminals
- First
	- source ros.sh
	- roslaunch robot_description sim_w1.launch
- Second
	- source ros.sh
	- roslaunch robot_description my_bug0_ros2.launch
- Third
	- source ros12.sh
	- ros2 run ros1_bridge dynamic_bridge
- Fourth
	- source ros2.sh
	- ros2 run ass2_nodes service
- Fifth
	- source ros2.sh
	- ros2 run ass2_nodes teleop
