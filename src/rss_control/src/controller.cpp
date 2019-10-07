//
// Created by ignat on 07/10/2019.
//

#include "controller.hpp"

Controller::Controller() : nh_("~") {

    nh_.param<bool>("debug", debug, false);
    if (debug) {
        if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
            ros::console::notifyLoggerLevelsChanged();
        }
    }

    nh_.param<std::string>("path_in_topic", path_in_topic, "/path");
    nh_.param<double>("waypoint_radius", waypoint_radius, 0.1);
    nh_.param<int>("control_frequency", control_frequeny, 50);
    nh_.param<double>("steering_p", steering_p, 1);
    nh_.param<double>("steering_i", steering_i, 0);
    nh_.param<double>("steering_d", steering_d, 0.2);
    nh_.param<double>("forward_speed", forward_speed, 0.5);

    steering_pid.setCoefs(steering_p, steering_i, steering_d);

    // subscribers and publishers
    path_sub_ = nh_.subscribe<nav_msgs::Path>(path_in_topic, 1, &Controller::pathCallback, this);
    control_pub_ = nh_.advertise<geometry_msgs::Twist>("command", 10);
    next_point_pub_ = nh_.advertise<geometry_msgs::Point>("next_point", 10);

    got_path_ = false;
}


void Controller::pathCallback(nav_msgs::Path path_msg) {
    path_frame_ = path_msg.header.frame_id;
    waypoints_ = path_msg.poses;
    got_path_ = true;
}

double euclidDistance(tf::Vector3 a, geometry_msgs::Pose b) {
    return sqrt(pow((a.x() - b.position.x), 2) + pow((a.y() - b.position.y), 2));
}

void Controller::controlLoop() {
    // first get our current pose
    tf::StampedTransform transform;

    try {
        listener.lookupTransform(MAP_FRAME, ROBOT_FRAME, ros::Time(0), transform);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("controller: Couldn't get transform: %s",ex.what());
        ros::Duration(1.0).sleep();
    }

    geometry_msgs::Twist control_msg;

    if (waypoints_.empty()) {
        ROS_DEBUG("No more waypoints. Maybe reached goal?");
    } else {
        // get the next point for which we are aiming for
        auto next_waypoint = waypoints_.front();

        // see if we have reached said point
        auto dist = euclidDistance(transform.getOrigin(), next_waypoint.pose);
        if (dist < waypoint_radius) {
            // remove first entry
            waypoints_.erase(waypoints_.begin());
        } else {
            // set constant forward velocity
            control_msg.linear.x = forward_speed;

            // do PID for the steering
            geometry_msgs::PoseStamped next_waypoint_transformed;
            listener.transformPose(ROBOT_FRAME, next_waypoint, next_waypoint_transformed);
            auto error = pow(next_waypoint_transformed.pose.position.x, 2);
            control_msg.angular.z = steering_pid.compute(error);
        }
        next_point_pub_.publish(next_waypoint);
    }
    control_pub_.publish(control_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    Controller controller;

    // Start CAN bus
    ros::Rate rate(controller.control_frequeny);

    while (ros::ok())
    {
        controller.controlLoop();
        ros::spinOnce();
        rate.sleep();
    }
}

