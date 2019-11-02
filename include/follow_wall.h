#ifndef FOLLOW_WALL_H
#define FOLLOW_WALL_H

#include <iostream>

#include <ros/package.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

class FollowWall {
 private:
   ros::Subscriber sensor_sub;
   ros::Publisher velocity_pub;
   ros::Subscriber odometry_sub;

   static const float ideal_distance = 0.05; //ideal distance the center of the robot should stay from the wall (in meters)
   static const float movement_speed = 0.2;

   float last_odometry_position[2];
   float last_odometry_yaw;
   static const float map_width = 10;
   static const float map_height = 10;
   bool firstOdometry;
 public:
    FollowWall(char**);
    void SensorHandler(const sensor_msgs::LaserScan& msg);
    void OdometryHandler(const nav_msgs::Odometry& msg);
    bool AtMapLimit(float);
};

#endif