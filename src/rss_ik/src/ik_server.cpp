#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "rss_ik/GetTargetJointAngles.h"
#include <cmath>
#include <vector>

using namespace std;
namespace rss {

    class JointAngles {
    public:
        double waist;
        double shoulder;
        double elbow;
        double wrist;
        double gripper;

        JointAngles(double waist, double shoulder, double elbow, double wrist) :
                waist(waist), shoulder(shoulder),
                elbow(elbow), wrist(wrist), gripper(0.0) {}

        JointAngles(double waist, double shoulder, double elbow, double wrist, double gripper) :
                waist(waist), shoulder(shoulder),
                elbow(elbow), wrist(wrist), gripper(gripper) {}

        std_msgs::Float64MultiArray getMessage(double gripperAngle) {
            std_msgs::Float64MultiArray msg;
            vector<double> angles;
            angles.reserve(6);
            angles.push_back(0.0);
            angles.push_back(waist);
            angles.push_back(shoulder);
            angles.push_back(elbow);
            angles.push_back(wrist);
            angles.push_back(gripperAngle);
            msg.data = angles;
            return msg;
        }

        std_msgs::Float64MultiArray getMessage() {
            return getMessage(gripper);
        }


    };

    std_msgs::Float64MultiArray current;
    bool needsUpdate = false;

    double normalizeAngle(const double &theta) {
        return theta - 2.0 * M_PI * floor((theta + M_PI) / (2.0 * M_PI));
    }

    bool doIk(rss_ik::GetTargetJointAngles::Request &req,
              rss_ik::GetTargetJointAngles::Response &res) {
        const double l1 = 0.04225;
        const double l2 = 0.105948101;
        const double l3 = 0.1;
        const double l4 = 0.063 + 0.0055;

        const double elbowOffset = 0.336674;

        const double theta1 = atan2(req.y, req.x);

        double x(sqrt(req.x * req.x + req.y * req.y));
        double y(req.z - l1);
        const double phi = req.phi;

        ROS_INFO("x: %f", x);
        ROS_INFO("y: %f", y);
        ROS_INFO("phi: %f", phi);

        x = x - l4 * cos(phi);
        y = y - l4 * sin(phi);

        const double xysumsq = x * x + y * y;
        const double xymagnitude = sqrt(xysumsq);

        const double gamma = atan2(-y / xymagnitude, -x / xymagnitude);


        ROS_INFO("x: %f", x);
        ROS_INFO("y: %f", y);
        ROS_INFO("xysumsq: %f", xysumsq);
        ROS_INFO("xymagnitude: %f", xymagnitude);
        ROS_INFO("gamma: %f", gamma);

        const double sigma = -1;

        const double acosInput = -(xysumsq + l2 * l2 - l3 * l3) / (2 * l2 * xymagnitude);
        double theta2 = gamma + sigma * acos(acosInput);

        const double temp1 = (y - l2 * sin(theta2)) / l3;
        const double temp2 = (x - l2 * cos(theta2)) / l3;
        double theta3 = atan2(temp1, temp2) - theta2;

        double theta4 = phi - theta3 - theta2;

        ROS_INFO("acos in: %f", acosInput);
        ROS_INFO("1: %f", theta1);
        ROS_INFO("2: %f", theta2);
        ROS_INFO("3: %f", theta3);
        ROS_INFO("4: %f", theta4);

        ROS_INFO("OLD[wa %f, sh %f , el %f , wr %f]", theta1, theta2, theta3, theta4);

        theta2 += -M_PI_2 + elbowOffset;
        theta3 += M_PI_2 - elbowOffset;
        theta2 = normalizeAngle(theta2);
        theta3 = normalizeAngle(theta3);

        ROS_INFO("NEW[wa %f, sh %f , el %f , wr %f]", theta1, theta2, theta3, theta4);

        if (isnan(theta1) || isnan(theta2) || isnan(theta3) || isnan(theta4)
            || theta2 > M_PI_2 || theta3 > M_PI_2 || theta4 > M_PI_2
            || theta2 < -M_PI_2 || theta3 < -M_PI_2 || theta4 < -M_PI_2) {
            res.valid = false;
            return false;
        }
        JointAngles solution(theta1, theta2, theta3, theta4, req.gripper);
        res.angles = solution.getMessage();
        current = solution.getMessage();
        res.valid = true;
        needsUpdate = true;
        return true;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ik_server");
    ros::NodeHandle n("~");
    ros::ServiceServer service = n.advertiseService("/get_target_joint_angles", rss::doIk);
//    ros::Publisher jointPub = n.advertise<std_msgs::Float64MultiArray>("/joint_trajectory_point", 4);
    ROS_INFO("Starting IK Server");
//    while (ros::ok()) {
//        ros::spinOnce();
//        if (rss::needsUpdate) {
//            jointPub.publish(rss::current);
//            rss::needsUpdate = false;
//        }
//
//    }
    ros::spin();
    return 0;
}

