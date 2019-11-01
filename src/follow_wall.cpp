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
    geometry_msgs::Twist vel_msg;

    float angle_current;
    /*float right_obstacle_min_distance = INFINITY, right_obstacle_angle = 0;
    float left_obstacle_min_distance = INFINITY, left_obstacle_angle = 0;
    float front_obstacle_min_distance = INFINITY;
    */
    float min_distance = INFINITY, alpha;

    for(int i=0; i< msg.ranges.size(); i++) {
        angle_current = rad2deg(msg.angle_min + msg.angle_increment * i);

        if(msg.ranges[i] < min_distance)
        {
            min_distance = msg.ranges[i];
            alpha = 90 - std::abs(angle_current);
        }
        /*if(angle_current < 0) //Right side
        {
            if(msg.ranges[i] < right_obstacle_min_distance)
            {
                right_obstacle_min_distance = msg.ranges[i];
                right_obstacle_angle = angle_current;
            }   
        } else { //Left side and front 
            if(msg.ranges[i] < left_obstacle_min_distance)
            {
                left_obstacle_min_distance = msg.ranges[i];
                left_obstacle_angle = angle_current;
            } 
        }*/
    }


    if(min_distance < msg.range_max) //found some obstacle
    {   
        float k = 25;
        vel_msg.linear.x = movement_speed;
		vel_msg.angular.z = (-k * (std::sin(deg2rad(alpha)) - (min_distance - ideal_distance))) * vel_msg.linear.x;
    } else { //no walls -> wander to find wall
        vel_msg.linear.x = 0.3;
		vel_msg.angular.z = 0.1;
    }

    velocity_pub.publish(vel_msg);
}