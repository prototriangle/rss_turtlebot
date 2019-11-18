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

  //Done following https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2

#include "a_star.hpp"
#include "utils.cpp"
#include <math.h>

AStar::AStar() : nh_("~") {

    bool debug;
    nh_.param<bool>("debug", debug, false);
    if (debug) {
        if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
        }
    }

    nh_.param<std::string>("odom_in_topic", odom_in_topic, "/gt_pose");
    nh_.param<std::string>("map_in_topic", map_in_topic, "/map");
    nh_.param<std::string>("target_point_topic", target_point_topic, "/clicked_point");
    nh_.param<float>("robot_radius", robot_radius, 0.25);

    ROS_DEBUG("Got parameter odom_in_topic %s", odom_in_topic.c_str());
    ROS_DEBUG("Got parameter map_in_tomap_resolution_pic %s", map_in_topic.c_str());
    ROS_DEBUG("Got parameter target_point_topic %s", target_point_topic.c_str());
    ROS_DEBUG("Got parameter robot_radius %f", robot_radius);

    // subscribers and publishers
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_in_topic, 1, &AStar::odomCallback, this);
    map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(map_in_topic, 1, &AStar::mapCallback, this);
    target_point_sub_ = nh_.subscribe<geometry_msgs::PointStamped>(target_point_topic, 1, &AStar::targetCallback, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 10, true);
    viz_pub_ = nh_.advertise<visualization_msgs::Marker>("path_viz", 10);

    got_map_ = false;
    got_odom_ = false;

  marker_msg.header.frame_id = MAP_FRAME;
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::POINTS;
  marker_msg.action = visualization_msgs::Marker::ADD;
  marker_msg.pose.orientation.w = 1.0;
  marker_msg.scale.x = 0.01;
  marker_msg.scale.y = 0.01;
  marker_msg.scale.z = 0.01;
  marker_msg.color.a = 0.4; // Don't forget to set the alpha!
  marker_msg.color.r = 0.2;
  marker_msg.color.g = 0.2;
  marker_msg.color.b = 0.8;
}

AStar::~AStar()
{
    odom_sub_.shutdown();
    map_sub_.shutdown();
    path_pub_.shutdown();
    nh_.shutdown();
}

void AStar::mapCallback(nav_msgs::OccupancyGrid map_msg) {
    ROS_DEBUG("Go OccupancyGrid message. Storing map..");
    // get map metadata
    map_resolution_ = map_msg.info.resolution;
    map_height = map_msg.info.height;
    map_width = map_msg.info.width;
    ROS_DEBUG("map resolution: %f", map_resolution_);
    ROS_DEBUG("map width: %d   height: %d", map_width, map_height);

    publish_frame_ = map_msg.header.frame_id;

    // convert map to eigen matrix
    // Data coming in is in row-major order aka. rows first starting from 0,0
    map = Eigen::MatrixXd(map_width, map_height);
    for (int i = 0; i < map_msg.data.size(); i++) {
        int row = i / map_width;
        int column = i % map_width;
        map(row, column) = map_msg.data[i];
    }

    ROS_DEBUG("map loaded OK");
    got_map_ = true;
}

void AStar::odomCallback(nav_msgs::Odometry odom_msg) {
    if (!got_map_) {
        return;
    }

    // convert position from real coordinates to discrete coordinates
    // NB. Converts floats to ints .. bad stuff might happen
    x_pos_ = odom_msg.pose.pose.position.x;
    y_pos_ = odom_msg.pose.pose.position.y;
    pixel_position_.x = toPixel(odom_msg.pose.pose.position.x);
    pixel_position_.y = toPixel(odom_msg.pose.pose.position.y);

    got_odom_ = true;
}

void AStar::targetCallback(geometry_msgs::PointStamped point_msg) {
    ROS_INFO("Got target point. Starting planning...");

    if (!got_map_) {
        ROS_DEBUG("Still don't have the map. Quitting..");
        return;
    }
    if (!got_odom_) {
        ROS_DEBUG("Still don't have position. Quitting..");
        return;
    }

    // check if target location is reachable in the first place
    if (!checkTargetLocation(point_msg.point.x, point_msg.point.y)) {
        ROS_ERROR("Target location requested is not reachable on the map");
        return;
    }

    // convert target position to pixels
    target_x_pos_ = (int)(point_msg.point.x / map_resolution_);
    target_y_pos_ = (int)(point_msg.point.y / map_resolution_);
    PixelPosition target_position = {target_x_pos_, target_y_pos_};

    /********** Now do the A* algorithm **********/
    // Initialise lists
    std::vector<Node> open_list;
    std::vector<Node> closed_list;
    Node start_node;
    start_node.position = pixel_position_;
    PixelPosition empty = {-999, -999};
    start_node.parent = empty;
    open_list.push_back(start_node);

    std::vector<PixelPosition> path;

    ROS_DEBUG("Starting A* algorithm calculation");
    ROS_DEBUG("From position %d, %d", start_node.position.x, start_node.position.y);

    while (!open_list.empty()) {
        // get current node
        auto current_node = getNodeMinimumF(open_list);
        closed_list.push_back(current_node);

        // check if we reached goal_node
        if (current_node.position == target_position) {
            ROS_DEBUG("Yaaas! Reached target node. Starting path generation");
            ROS_DEBUG("Current node: %d %d   Target node: %d %d", current_node.position.x, current_node.position.y, target_position.x, target_position.y);

            auto current = current_node;
            // check if we have reached starting node
            while (!((current.parent.x == -999) && (current.parent.y == -999))) {
                path.push_back(current.position);
                current = findParentNode(current.parent, closed_list);
            }
            ROS_DEBUG("Finished path generation");
            break;
        }

        // not reached goal; generate new paths
        ROS_DEBUG("Generating new candidate paths");
        for (int i=-1; i<2; i++) {
            for (int j=-1; j<2; j++) {
              if (!((i == 0) && (j == 0))) {
                // generate new position
                PixelPosition new_position = {current_node.position.x + i, current_node.position.y + j};
                ROS_DEBUG("Trying node with position %d, %d", current_node.position.x, current_node.position.y);

                // ensure that node is reachable
                if (checkTargetLocation(toEuclid(new_position.x), toEuclid(new_position.y))) {
                  Node new_node;
                  new_node.position = new_position;
                  new_node.parent = current_node.position;

                  // check if the new node hasn't been explored yet
                  if (checkIfNodeInList(new_node, closed_list) == 99999) {
                    ROS_DEBUG("Node is reachable and not explored previously");
                    // create score (aka cost)
                    new_node.g_score = current_node.g_score + 1;
                    new_node.h_score = nodeCost(new_node.position, target_position);
                    new_node.f_score = new_node.g_score + new_node.h_score;
                    ROS_DEBUG("Node f-score is %d", new_node.f_score);

                    // add child node to open list
                    auto old_node_g_score = checkIfNodeInList(new_node, open_list);
                    if (old_node_g_score == 99999) {
                      ROS_DEBUG("Node is decent new candidate. Adding to open list");
                      open_list.push_back(new_node);
                    }
                  }
                }
              }
            }
        }
        if (viz_pub_.getNumSubscribers() > 0) {
          publishVisualisation(closed_list);
        }
    }

    ROS_INFO("Finished A* algorithm. Now publishing path");

    if (!path.empty()) {
        // we actually have a good path
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = MAP_FRAME;
        path_msg.header.stamp = ros::Time::now();
          for (int i=path.size()-1; i>0; i=i-5) {
            geometry_msgs::PoseStamped geo_point;
            geo_point.header.frame_id = MAP_FRAME;
            geo_point.pose.position.x = (float)path[i].x * map_resolution_;
            geo_point.pose.position.y = (float)path[i].y * map_resolution_;
            geo_point.pose.orientation.w = 1.0;
            path_msg.poses.push_back(geo_point);
        }
        path_pub_.publish(path_msg);
        ROS_DEBUG("A* path is published");
    } else {
        ROS_ERROR("A* path generation returned empty path");
    }
}

bool AStar::publishVisualisation(std::vector<Node> &nodes) {
  marker_msg.action = visualization_msgs::Marker::DELETE;
  marker_msg.header.stamp = ros::Time::now();
  viz_pub_.publish(marker_msg);

  marker_msg.points.clear();
  marker_msg.action = visualization_msgs::Marker::ADD;

  for (const auto node : nodes) {
    geometry_msgs::Point geo_pt;
    geo_pt.x = toEuclid(node.position.x);
    geo_pt.y = toEuclid(node.position.y);
    marker_msg.points.push_back(geo_pt);
  }
  viz_pub_.publish(marker_msg);
}

bool AStar::checkTargetLocation(double target_x, double target_y) {
    ROS_DEBUG("Checking target position validity");

    if ((target_x < 0) || (target_y < 0)) {
        ROS_DEBUG("Requested point has negative coordinates  x=%f y=%f", target_x, target_y);
        return false;
    }

    std::vector<double> angles = linspace(0., 2*M_PI, 50.);

    for (auto angle: angles) {
        // compute point around the circumference of the robot
        double x_to_check = robot_radius*sin(angle) + target_x;
        double y_to_check = robot_radius*cos(angle) + target_y;

        ROS_DEBUG("Checking point %f, %f in circumference with angle %f", x_to_check, y_to_check, angle);

        // first check if the target fits in the map
        if ((toPixel(x_to_check) >= map_width) || (toPixel(y_to_check) >= map_height)
             || (toPixel(x_to_check) < 0) || (toPixel(y_to_check) < 0)) {
            ROS_DEBUG("Requested point is not within the map");
            return false;
        }

        // now check pixel value on costmap and see if it is OK
        ROS_DEBUG("Checking map value of %d %d with angle %f", toPixel(x_to_check), toPixel(y_to_check), angle);
        // x and y axis are swapped for some unknown reason
        auto map_value = map(toPixel(y_to_check), toPixel(x_to_check));
        if (map_value != 0) {
          ROS_DEBUG("Obstacle in path. Not valid point");
          return false;
        }
    }
    return true;
}

Node AStar::getNodeMinimumF(std::vector<Node> &nodes) {
    int idx = INT_MAX; // index of best node
    int min_cost = INT_MAX; // starting cost
    for (int i=0; i<nodes.size(); ++i) {
        if (nodes[i].f_score < min_cost) {
            min_cost = nodes[i].f_score;
            idx = i;
        }
    }
    auto best_node = nodes[idx];
    nodes.erase(nodes.begin() + idx);
    return best_node;
}

Node AStar::findParentNode(PixelPosition position, std::vector<Node> &nodes) {
    for (const auto& node: nodes) {
        if (node.position == position)
            return node;
    }
}

int AStar::checkIfNodeInList(Node searched_node, std::vector<Node> &nodes) {
    for (int i=0; i < nodes.size(); i++) {
        if (nodes[i].position == searched_node.position)
            return nodes[i].g_score;
    }
    return 99999;
}

int AStar::nodeCost(PixelPosition node_position, PixelPosition target_position) {
    return pow((node_position.x - target_position.x),2) + pow((node_position.y - target_position.y), 2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "a_star");
    AStar planner;
    ros::spin();
}

