#include "../include/follow_wall.h"

FollowWall::FollowWall(char** argv) {
    ros::NodeHandle nh;

    //publish on robot movement topic
    std::string speed_topic = std::string("/") + std::string(argv[1]) + std::string("/cmd_vel");
    velocity_pub = nh.advertise<geometry_msgs::Twist>(speed_topic, 5);
    
    //subscribe laser topic
    std::string laser_topic = std::string("/") + std::string(argv[1]) + std::string("/") + std::string(argv[2]);
    sensor_sub = nh.subscribe(laser_topic.c_str(), 5, &FollowWall::SensorHandler, this);
}

void FollowWall::SensorHandler(const sensor_msgs::LaserScan& msg) {
    //Handle laser sensor messages
   ROS_INFO_STREAM(msg.angle_min);
}