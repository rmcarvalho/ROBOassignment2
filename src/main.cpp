#include "../include/follow_wall.h"

int main(int argc, char** argv)
{
    if(argc != 3) {
        std::cout << "Usage: rosrun robo_assign2 follow_wall_robot_node <robot_frame_id> <laser_frame_id>\n";
        return -1;
    }

    ros::init(argc, argv, "robot_follow_wall", ros::init_options::AnonymousName);
    FollowWall obj = FollowWall(argv);
    ros::spin();
    return 0;
}