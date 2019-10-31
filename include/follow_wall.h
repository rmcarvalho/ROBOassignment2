#ifndef FOLLOW_WALL_H
#define FOLLOW_WALL_H

#include <iostream>

#include <ros/package.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class FollowWall {
 private:
    ros::Subscriber sensor_sub;
    ros::Publisher velocity_pub;
    
 public:
    FollowWall(char**);
    void SensorHandler(const sensor_msgs::LaserScan& msg);
};

#endif