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

    nh_.param<std::string>("path_in_topic", path_in_topic, "/a_star/path");
    nh_.param<double>("waypoint_radius", waypoint_radius, 0.1);
    nh_.param<int>("control_frequency", control_frequeny, 50);
    nh_.param<double>("steering_p", steering_p, 1);
    nh_.param<double>("steering_i", steering_i, 0);
    nh_.param<double>("steering_d", steering_d, 0.2);
    nh_.param<double>("forward_speed", forward_speed, 0.3);

    steering_pid.setCoefs(steering_p, steering_i, steering_d);
    steering_pid.setConstraints(-1, 1);

    // subscribers and publishers
    path_sub_ = nh_.subscribe<nav_msgs::Path>(path_in_topic, 1, &Controller::pathCallback, this);
    control_pub_ = nh_.advertise<geometry_msgs::Twist>("command", 10);
    next_point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("next_point", 10);
    status_pub_ = nh_.advertise<std_msgs::Int16>("status", 1, true);

    got_path_ = false;

    // set status to 0 meaning that it's booted up
    std_msgs::Int16 status_msg;
    status_msg.data = 0;
    status_pub_.publish(status_msg);
}


void Controller::pathCallback(nav_msgs::Path path_msg) {
    ROS_INFO("Controller: Received path");
    path_frame_ = path_msg.header.frame_id;
    waypoints_ = path_msg.poses;
    got_path_ = true;
    std_msgs::Int16 status_msg;
    status_msg.data = 1; // set status to executing
    status_pub_.publish(status_msg);
}

double euclidDistance(tf::Vector3 a, geometry_msgs::Pose b) {
    return sqrt(pow((a.x() - b.position.x), 2) + pow((a.y() - b.position.y), 2));
}

void Controller::controlLoop() {
    if (!got_path_) {
        ROS_DEBUG("Controller: Haven't received path yet...");
        ros::Duration(1.0).sleep();
        return;
    }

    // first get our current pose
    tf::StampedTransform transform;

    try {
        listener.lookupTransform(path_frame_, ROBOT_FRAME, ros::Time(0), transform);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("controller: Couldn't get transform: %s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    geometry_msgs::Twist control_msg;

    if (waypoints_.empty()) {
        ROS_DEBUG("No more waypoints. Maybe reached goal?");
        std_msgs::Int16 status_msg;
        status_msg.data = 2; // set status to finished
        status_pub_.publish(status_msg);
        ros::Duration(1.0).sleep();
    } else {

        // get the next point for which we are aiming for
        ROS_DEBUG("Have %d waypoints left", (int)waypoints_.size());
        auto next_waypoint = waypoints_.front();
        ROS_DEBUG("Next waypoint is %f %f", next_waypoint.pose.position.x, next_waypoint.pose.position.y);

        // see if we have reached said point
        auto dist = euclidDistance(transform.getOrigin(), next_waypoint.pose);
        if (dist < waypoint_radius) {
            // remove first entry
            ROS_DEBUG("Reached waypoint");
            waypoints_.erase(waypoints_.begin());

        } else {
            // do PID for the steering
            geometry_msgs::PoseStamped next_waypoint_transformed;
            listener.transformPose(ROBOT_FRAME, next_waypoint, next_waypoint_transformed);
            auto error = next_waypoint_transformed.pose.position.y;
            control_msg.angular.z = steering_pid.compute(error);
            ROS_DEBUG("Steering error %f", error);

	    // now calculate the forward speed based on my empirical function here
	    if (fabs(control_msg.angular.z) > forward_speed/2) {
		if (fabs(control_msg.angular.z) > forward_speed) {
		  control_msg.linear.x = 0;
		} else {
		  control_msg.linear.x = forward_speed - control_msg.angular.z;
		}
	    } else {
		control_msg.linear.x = forward_speed;
	    }
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

