# ROBOassignment2

https://www.overleaf.com/2417578735rqhtymrhhhpx


## Installation

* Setup ros workspace: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
* Install STDR from Github into it's own package in the workspace - http://wiki.ros.org/stdr_simulator/Tutorials/Set%20up%20STDR%20Simulator

 * In the 'src' folder create package 'robo_assign2' with command
```
catkin_create_pkg robo_assign2
```
 * Clone this repository into the 'robo_assign2' folder
 * Copy the files in robo_assign2/stdr_files to the stdr_simulator package

 * Build the package in the workspace folder by running:
```
catkin_make
```

## Running

 * On any terminal you use, run 
 
 ```source devel/setup.bash```

 * Open a terminal and run:
 
```
roscore
```


 * Open a terminal and choose a map:
```
roslaunch stdr_launchers assignment2_d20_a<angle>.launch <- angle can be one of: 0 45 90 135 180
```

 * Open a second terminal and deploy robot:
```
rosrun robo_assign2 follow_wall_robot_node <robot_frame_id> <laser_frame_id>
```
for example:
```
rosrun robo_assign2 follow_wall_robot_node robot0 laser0
```
