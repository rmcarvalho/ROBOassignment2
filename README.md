# ROBOassignment1

## Development

Setup ros workspace and in the 'src' folder create package 'robo_assign2' with command
```
catkin_create_pkg robo_assign2
```
Clone this repositoy into the 'robo_assign2' folder

To deploy robot, run:
```
rosrun robo_assign2 follow_wall_robot_node <robot_frame_id> <laser_frame_id>
```

for example:
```
rosrun robo_assign2 follow_wall_robot_node robot_0 laser_0
```
