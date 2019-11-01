//
// Created by ignat on 07/10/2019.
//

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_path");
    ros::NodeHandle nh("~");

    // create waypoints
    geometry_msgs::PoseStamped geo_pt1;
    geo_pt1.header.frame_id = "odom";
    geo_pt1.pose.position.x = 1.0;
    geo_pt1.pose.orientation.w = 1.0;


    geometry_msgs::PoseStamped geo_pt2;
    geo_pt2.header.frame_id = "odom";
    geo_pt2.pose.position.x = 1.0;
    geo_pt2.pose.position.y = -1.0;
    geo_pt2.pose.orientation.w = 1.0;

    geometry_msgs::PoseStamped geo_pt3;
    geo_pt3.header.frame_id = "odom";
    geo_pt3.pose.position.x = 2.0;
    geo_pt3.pose.position.y = -1.0;
    geo_pt3.pose.orientation.w = 1.0;

    // create path
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "odom";
    path_msg.header.stamp = ros::Time::now();
    path_msg.poses.push_back(geo_pt1);
    path_msg.poses.push_back(geo_pt2);
    path_msg.poses.push_back(geo_pt3);

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path", 1, true);
    path_pub.publish(path_msg);
    ros::spin();
}
