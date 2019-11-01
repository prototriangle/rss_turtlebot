/*
* MIT License
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
*         AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#ifndef RSS_CONTROL_A_STAR_H
#define RSS_CONTROL_A_STAR_H

#define MAP_FRAME "map"
#define ROBOT_FRAME "base_footprint"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>


struct PixelPosition {int x; int y;};

struct Node {PixelPosition position;
    PixelPosition parent;
    int f_score=0;
    int g_score=0;
    int h_score=0;
};

bool operator==(const PixelPosition& lhs, const PixelPosition& rhs)
{
    return ((lhs.x == rhs.x) && ( lhs.y == rhs.y));
}


class AStar {
public:
    AStar(); ///< Constructor
    ~AStar(); ///< Destructor

    std::string odom_in_topic;
    std::string map_in_topic;
    std::string target_point_topic;
    float robot_radius;
  visualization_msgs::Marker marker_msg;

private:

    void odomCallback(nav_msgs::Odometry odom_msg);
    void mapCallback(nav_msgs::OccupancyGrid map_msg);
    void targetCallback(geometry_msgs::PointStamped point_msg);

    bool checkTargetLocation(double target_x, double target_y);
    Node getNodeMinimumF(std::vector<Node> &nodes);
    Node findParentNode(PixelPosition position, std::vector<Node> &nodes);
    int checkIfNodeInList(Node searched_node, std::vector<Node> &nodes);
    int nodeCost(PixelPosition node_positoin, PixelPosition target_position);
  bool publishVisualisation(std::vector<Node> &nodes);

    int toPixel(double val) {
        return val/map_resolution_;
    }

    double toEuclid(int val) {
        return val * map_resolution_;
    }

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber target_point_sub_;
    ros::Publisher path_pub_;
    ros::Publisher viz_pub_;

    float map_resolution_;
    int map_width, map_height;
    bool got_map_, got_odom_;

    Eigen::MatrixXd map;

    double x_pos_, y_pos_;
    int x_pos_pixels_, y_pos_pixels_;
    int target_x_pos_, target_y_pos_;
    PixelPosition pixel_position_;

    std::string publish_frame_; // This is the frame in which to publish stuff
};

#endif //RSS_CONTROL_A_STAR_H
