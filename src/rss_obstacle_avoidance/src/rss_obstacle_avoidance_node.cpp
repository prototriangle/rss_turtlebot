#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "ObstacleAvoidance.h"
#include <vector>


using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace std;

LaserScan lastScan;
bool scanValid = false;
bool newLidar = false;
Twist currentTwist;
bool lockDirection = false;

const unsigned long aheadLook[] = {320, 330, 340, 350, 0, 10, 20, 30, 40};
const unsigned long extendedLook[] = {260, 270, 280, 290, 300, 310, 320, 330, 340, 350, 0, 10, 20, 30, 40, 50, 60, 70,
                                      80, 90, 100};
const unsigned long leftAheadLook[] = {270, 280, 290, 300};
const unsigned long rightAheadLook[] = {60, 70, 80, 90};
const unsigned long leftLook[] = {240, 250, 260, 270, 280, 290, 300};
const unsigned long rightLook[] = {60, 70, 80, 90, 100, 110, 120};

Vector3 defaultLinearVel;

void handleScan(const LaserScan::ConstPtr &msg) {
    lastScan = *msg;
    scanValid = true;
    newLidar = true;
}

bool collisionIsImminent() {
    const float thresh = 0.45;
    for (auto &i : aheadLook) {
        const auto val = lastScan.ranges[i];
        if (val > lastScan.range_min && val < thresh && val >= 0.015)
            return true;
    }
    return false;
}

float sumLeft() {
    float sum = 0;
    for (auto &i : leftAheadLook) {
        const auto val = lastScan.ranges[i];
        if (val > lastScan.range_min && val < lastScan.range_max)
            sum += val;
    }
    return sum;
}

float sumRight() {
    float sum = 0;
    for (auto &i : rightAheadLook) {
        const auto val = lastScan.ranges[i];
        if (val > lastScan.range_min && val < lastScan.range_max)
            sum += val;
    }
    return sum;
}

void preventCollision() {
    static const float turnSpeed = M_2_PI / 1.5f;
    static float lastTurn;
    auto r = sumRight();
    auto l = sumLeft();
    if (!lockDirection) {
      lockDirection = true;
      if (r > l) {
          currentTwist.angular.z = turnSpeed;
          lastTurn = turnSpeed;
      } else {
          currentTwist.angular.z = -turnSpeed;
          lastTurn = -turnSpeed;
      }
    } else {
      currentTwist.angular.z = lastTurn;
    }
    //currentTwist.angular.z = (r - l) / (r + l) * turnSpeed;
    currentTwist.linear.x = 0.05;

    ROS_INFO("Prevent Collision %f", currentTwist.angular.z);
}

void turnTowardsMostDistantAhead() {
//    ROS_INFO("Turn Towards Most Distant");
    const float fastTurn = M_1_PI / 1.5f;
    const float slowTurn = M_1_PI / 3.0f;
    unsigned long maxI = 0;
    float max = -1.0f;
    for (auto &i : extendedLook) {
        const float val = lastScan.ranges[i];
        if (val < lastScan.range_max && val > lastScan.range_min && val > max) {
            max = val;
            maxI = i;
        }
    }
    if (maxI >= 200 && maxI <= 290) {
        ROS_INFO("Left!");
        currentTwist.angular.z = -fastTurn;
    } else if (maxI >= 290 && maxI <= 340) {
        ROS_INFO("Left.");
        currentTwist.angular.z = -slowTurn;
    } else if (maxI <= 160 && maxI >= 70) {
        ROS_INFO("Right!");
        currentTwist.angular.z = fastTurn;
    } else if (maxI <= 70 && maxI >= 20) {
        currentTwist.angular.z = slowTurn;
        ROS_INFO("Right.");
    } else {
        ROS_INFO("Straight");
        currentTwist.angular = Vector3();
    }
    ROS_INFO("Angular: %f", currentTwist.angular.z);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rss_obstacle_avoidance_node");
    ros::NodeHandle n;
    ros::Subscriber scanSub = n.subscribe("scan", 4, handleScan);
    ros::Publisher cmdPub = n.advertise<Twist>("/avoidance/cmd", 2);

    defaultLinearVel.x = 0.26;
    currentTwist.linear = defaultLinearVel;
    currentTwist.angular = Vector3();
    while (ros::ok()) {
        ros::spinOnce();
        if (!scanValid || !newLidar)
            continue;
        if (collisionIsImminent()) {
            ROS_INFO("Collision Imminent");
            // IF collision imminent:
            // |  Move to prevent collision
            preventCollision();
        } else {
            // ELSE:
            // |  Set forward vel back to max
            // |  Set rotational vel to move towards furthest range
            lockDirection = false;
	    currentTwist.linear = defaultLinearVel;
            turnTowardsMostDistantAhead();
        }
        newLidar = false;
        cmdPub.publish(currentTwist);
        ROS_INFO("x: %f", currentTwist.linear.x);
    }

    return 0;
}
