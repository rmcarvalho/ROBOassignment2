# Directory Structure

	Ricardo_Carvalho_Vítor_Magalhães
	│
	├───include
	│
	├───src
	│
	└───stdr_files
		├───stdr_launchers
		│   └───launch
		│           
		└───stdr_resources
			├───maps
			│       
			└───resources
				└───robots

	The 'include' folder is has header files.
	The 'src' folder has the c++ source files.
	The 'stdr_files' folder has all the files to be used with the simulator (launcher files, maps as PNG and YAML files and a robot as YAML and XML files)

# Requirements

	This package requires:
		- ROS Kinetic (available at: http://wiki.ros.org/kinetic/Installation)
		- STDR simulator installed into its own package in the workspace (available at: http://wiki.ros.org/stdr_simulator/Tutorials/Set%20up%20STDR%20Simulator#Get_STDR_Simulator_from_Github)
		- the contents of this folder should be copied inte a package named 'robo_assign2', created with 'catkin_create_pkg robo_assign2'
		- the contents of the 'stdr_files' folder should be copied into the 'stdr_simulator' folder
		- buiding the project requires c++11

# Compiling

	If all the requirements are met, the project can be built by running 'catkin_make' from the ROS workspace folder.

# Executing

	3 terminals are required to run the program:
		- The first terminal should run 'roscore'
		- Then we can open the simulator and choose a map, by opening a terminal in te workspace folder and running:

			$ source devel/setup.bash
			$ roslaunch stdr_launchers assignment2_d<thickness>_a<angle>.launch

			Where thickness is one of three values: {20, 40, 60} and angle is one of {0, 45, 90, 135, 180}
		- The third terminal should, from the workspace directory, run:

			$ source devel/setup.bash
			$ rosrun robo_assign2 follow_wall_robot_node robot0 laser0

			Where 'robot0' and 'laser0' are the names of the robot and the laser, respectively, in the simulator.
