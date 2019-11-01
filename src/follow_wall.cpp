#include "../include/follow_wall.h"

float rad2deg (float rad) {
    return rad * 180 / 3.14159;
}

float deg2rad (float deg) {
    return deg * 3.14159 / 180;
}

FollowWall::FollowWall(char** argv) {
    ros::NodeHandle nh;

    //publish on robot movement topic
    std::string speed_topic = std::string("/") + std::string(argv[1]) + std::string("/cmd_vel");
    velocity_pub = nh.advertise<geometry_msgs::Twist>(speed_topic, 1);
    
    //subscribe laser topic
    std::string laser_topic = std::string("/") + std::string(argv[1]) + std::string("/") + std::string(argv[2]);
    sensor_sub = nh.subscribe(laser_topic.c_str(), 1, &FollowWall::SensorHandler, this);
}

void FollowWall::SensorHandler(const sensor_msgs::LaserScan& msg) {
    float range_min = msg.range_min, range_max = msg.range_max;
    float angle_min = msg.angle_min, angle_max = msg.angle_max;
    float angle_step = msg.angle_increment;
    
    geometry_msgs::Twist vel_msg;

    float angle_current;
    float right_obstacle_min_distance = INFINITY, right_obstacle_angle = 0;
    float left_obstacle_min_distance = INFINITY, left_obstacle_angle = 0;
    float front_obstacle_min_distance = INFINITY;
    
    float min_distance = INFINITY, alpha;

    for(int i=0; i< msg.ranges.size(); i++) {
        angle_current = rad2deg(msg.angle_min + msg.angle_increment * i);

        if(msg.ranges[i] < min_distance)
        {
            min_distance = msg.ranges[i];
            alpha = 90 - std::abs(angle_current);
        }

        if(angle_current < -15) //Right side
        {
            if(msg.ranges[i] < right_obstacle_min_distance)
            {
                right_obstacle_min_distance = msg.ranges[i];
                right_obstacle_angle = angle_current;
            }   
        } else if(angle_current > 15){ //Left side
            if(msg.ranges[i] < left_obstacle_min_distance)
            {
                left_obstacle_min_distance = msg.ranges[i];
                left_obstacle_angle = angle_current;
            } 
        } else { // 30deg of the front - detect when on the inside of a corner
            if(msg.ranges[i] < front_obstacle_min_distance)
            {
                front_obstacle_min_distance = msg.ranges[i];
            } 
        }
    }

    

    //wall on left -> if at 90deg just take step else step and rotate
    //wall on right-> if at -90deg just take step else step and rotate
    //wall on front-> rotate to closest
    //wall left and right -> keep following wall closest on |90|deg
    if(min_distance < msg.range_max) //not infinity
    {   
        float k = 20;
        vel_msg.linear.x = movement_speed;
		vel_msg.angular.z = (-k * (std::sin(deg2rad(alpha)) - (min_distance - ideal_distance))) * vel_msg.linear.x;
    } else { //no walls -> wander to find wall
        vel_msg.linear.x = 0.3;
		vel_msg.angular.z = 0.1;
    }

//TODO
    //wall on left and front -> turn right to have front wall on 90deg

    //wall on right and front -> turn left to have front wall on -90deg
    
    //wall on left, right and front
        //given it's an inner corner when this happens check distance at -90 and 90 and turn towards the furthest one (the one not being followed)
    velocity_pub.publish(vel_msg);
}