//
// Created by ignat on 07/10/2019.
//

#ifndef RSS_CONTROL_CONTROLLER_H
#define RSS_CONTROL_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <string>

#include "PID.hpp"

#define MAP_FRAME "map"
#define ROBOT_FRAME "base_footprint"

class Controller {

public:
    Controller(); ///< Constructor

    std::string path_in_topic;
    bool debug;

    double waypoint_radius; ///< Radius around waypoint which marks it as reached
    int control_frequeny; ///< control loop frequency
    double steering_p, steering_i, steering_d;  ///< PID steering parameters
    double forward_speed;

    void controlLoop();

private:

    void pathCallback(nav_msgs::Path path_msg);

    PID steering_pid;

    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Publisher control_pub_;
    ros::Publisher next_point_pub_;
    tf::TransformListener listener;

    std::string path_frame_;
    std::vector<geometry_msgs::PoseStamped> waypoints_;
    bool got_path_;

};


#endif //RSS_CONTROL_CONTROLLER_H
