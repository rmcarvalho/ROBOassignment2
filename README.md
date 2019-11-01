# ROBOassignment1

## Installation

* Setup ros workspace: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
* Install STDR from Github into it's own package in the workspace - http://wiki.ros.org/stdr_simulator/Tutorials/Set%20up%20STDR%20Simulator

In the 'src' folder create package 'robo_assign2' with command
```
catkin_create_pkg robo_assign2
```
Clone this repository into the 'robo_assign2' folder

To deploy robot, run:
```
rosrun robo_assign2 follow_wall_robot_node <robot_frame_id> <laser_frame_id>
```

for example:
```
rosrun robo_assign2 follow_wall_robot_node robot0 laser0
```
