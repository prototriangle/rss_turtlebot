/*MIT License
*
* Copyright (c) 2019 Edinburgh University Formula Student (EUFS)
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
*         of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
*         to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*         copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
*         copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*         AUTHORS OR COPYRIGHT HOLDERS BE LIAmapBLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.*/

#include "a_star.hpp"
#include "utils.cpp"
#include <math.h>

AStar::AStar() : nh_("~")
{

    nh_.param<std::string>("odom_in_topic", odom_in_topic);
    nh_.param<std::string>("map_in_topic", map_in_topic);
    nh_.param<std::string>("target_point_topic", target_point_topic);
    nh_.param<float)("robot_radius", robot_radius, 0.25)

    if (odom_in_topic.empty() or map_in_topic.empty(), target_point_topic.empty()) {
        ROS_ERROR("Input topics are not defined. Qutting..");
        exit(1);
    }

    // subscribers and publishers    got_map_ = true;

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_in_topic, 1, AStar::odomCallback, this);
    map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(map_in_topic, 1, AStar::mapCallback, this);
    target_point_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(target_point_topic, 1, AStar::targetCallback, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 10);

    got_map_ = false;
    got_odom_ = false;
}

AStar::~AStar()
{
    odom_sub_.shutdown();
    map_sub_.shutdown();
    path_pub_.shutdown();
    nh_.shutdown();
}

void AStar::mapCallback(nav_msgs::OccupancyGrid &map_msg) {
    ROS_DEBUG("Go OccupancyGrid message. Storing map..");
    // get map metadata
    map_resolution_ = map_msg.info.resolution;
    map_height = map_msg.info.height;
    map_width = map_msg.info.width;
    ROS_DEBUG("map resolution: %f", map_resolution_);
    ROS_DEBUG("map width: %d   height: %d", map_width, map_height);

    // convert map to eigen matrix
    // Data coming in is in row-major order aka. rows first starting from 0,0
    map = Eigen::MatrixXd(map_width, map_height);
    for (int i = 0; i < map_msg.data.size(); i++) {
        int row = i / map_width;
        int column = i - row*map_width;
        map(row, column) = map_msg.data[i];
        ROS_DEBUG("Inserting data to row %d column %d", row, column);
    }

    got_map_ = true;
}

void AStar::odomCallback(nav_msgs::Odometry &odom_msg) {
    ROS_DEBUG("Starting odom callback");
    if (!got_map_) {
        ROS_DEBUG("Still don't have the map. Quitting..");
        return;
    }

    // convert position from real coordinates to discrete coordinates
    // NB. Converts floats to ints .. bad stuff might happen
    x_pos_ = odom_msg.pose.pose.position.x;
    y_pos_ = odom_msg.pose.pose.position.y;
    x_pos_pixels_ = (int)(odom_msg.pose.pose.position.x * map_resolution_);
    y_pos_pixels_ = (int)(odom_msg.pose.pose.position.y * map_resolution_);

    got_odom_ = true;
}

void AStar::targetCallback(geometry_msgs::Point &point_msg) {
    ROS_DEBUG("Starting target point callback");
    if (!got_map_) {
        ROS_DEBUG("Still don't have the map. Quitting..");
        return;
    }
    if (!got_odom_) {
        ROS_DEBUG("Still don't have position. Quitting..");
        return;
    }

    // check if target location is reachable in the first place
    if (!checkTargetLocation(point_msg.x, point_msg.y)) {
        ROS_ERROR("Target location requested is not reachable on the map");
        return;
    }

    // convert target position to pixels
    target_x_pos_ = (int)(point_msg.x * map_resolution_);
    target_y_pos_ = (int)(point_msg.y * map_resolution_);

    // Now do the A* algorithm
}

bool AStar::checkTargetLocation(int target_x, int target_y) {
    ROS_DEBUG("Checking target position validity");

    std::vector<double > angles = linspace(0., M_PI, 20.);

    for (auto angle: angles) {
        // compute point around the circumference of the robot
        double x_to_check = robot_radius*cos(angle) + x_pos_;
        double y_to_check = robot_radius*sin(angle) + y_pos_;

        ROS_DEBUG("Checking point %f, %f in circumference", x_to_check, y_to_check);

        // now check pixel value on costmap and see if it is OK
        auto map_value = map(x_to_check*map_width, y_to_check*map_height);
        if (map_value != 0) {
            return false;
        }
        return true;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "a_star");
    AStar planner;

    ros::spin();
}

