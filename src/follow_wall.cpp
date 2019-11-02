#include "../include/follow_wall.h"

float rad2deg (float rad) {
    return rad * 180 / 3.14159;
}

float deg2rad (float deg) {
    return deg * 3.14159 / 180;
}

FollowWall::FollowWall(char** argv) {
    ros::NodeHandle nh;
    firstOdometry = false;
    //publish on robot movement topic
    std::string speed_topic = std::string("/") + std::string(argv[1]) + std::string("/cmd_vel");
    velocity_pub = nh.advertise<geometry_msgs::Twist>(speed_topic, 1);
    
    //subscribe laser topic
    std::string laser_topic = std::string("/") + std::string(argv[1]) + std::string("/") + std::string(argv[2]);
    sensor_sub = nh.subscribe(laser_topic.c_str(), 1, &FollowWall::SensorHandler, this);

    //subscribe odometry topic
    std::string odom_topic = std::string("/") + std::string(argv[1]) + std::string("/odom");
    odometry_sub = nh.subscribe(odom_topic.c_str(), 1, &FollowWall::OdometryHandler, this);
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
    
    bool isOnMapLimit = AtMapLimit(msg.range_max);

    if(min_distance < msg.range_max && !isOnMapLimit) //found some obstacle
    {   
        ROS_INFO_STREAM("ON WALL");
        float k = 25;

        vel_msg.linear.x = movement_speed;
		vel_msg.angular.z = (-k * (std::sin(deg2rad(alpha)) - (min_distance - ideal_distance))) * vel_msg.linear.x;
        ROS_INFO_STREAM(min_distance - ideal_distance);
    } else { //no walls -> wander to find wall
        if(isOnMapLimit) {
            ROS_INFO_STREAM("Map LIMIT");
            vel_msg.linear.x = 0;
            vel_msg.angular.z = deg2rad(rand() % 180 + 90); // random
        } else {
            vel_msg.linear.x = movement_speed*2;
		    vel_msg.angular.z = 0;
        }
        
    }

    velocity_pub.publish(vel_msg);
}

void FollowWall::OdometryHandler(const nav_msgs::Odometry& msg) {
    last_odometry_position[0] = msg.pose.pose.position.x;
    last_odometry_position[1] = msg.pose.pose.position.y;
    geometry_msgs::Quaternion q = msg.pose.pose.orientation;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    last_odometry_yaw = std::atan2(siny_cosp, cosy_cosp);
    firstOdometry = true;
}

bool FollowWall::AtMapLimit(float range_max) {
    float min_distance_to_edge = range_max + 0.005;
    std::cout << firstOdometry << "\tat: " << last_odometry_position[0] << ", " << last_odometry_position[1] << "\trotated: " << last_odometry_yaw << "\tkeep distance: " << min_distance_to_edge << std::endl;
    return firstOdometry && ((last_odometry_position[0] <= min_distance_to_edge && std::abs(rad2deg(last_odometry_yaw)) > 90) ||
        (last_odometry_position[0] >= map_width - min_distance_to_edge && std::abs(rad2deg(last_odometry_yaw)) < 90) ||
        (last_odometry_position[1] <= min_distance_to_edge && std::sin(last_odometry_yaw) < std::sin(deg2rad(10))) ||
        (last_odometry_position[1] >= map_height - min_distance_to_edge && std::sin(last_odometry_yaw) > std::sin(deg2rad(-10))));

}