#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16.h"
#include "rss_ik/GetTargetJointAngles.h"
#include "rss_ik/SendIKCommand.h"
#include "rss_ik/Sequences.h"
#include "tf/transform_listener.h"
#include <utility>
#include <vector>

using namespace std;

namespace rss {
    ros::Publisher jointPub;
    ros::Publisher statusPub;
    ros::Subscriber armPosSub;
    ros::ServiceClient targetServiceClient;


    tf::TransformListener *listener;

    void processCommand(const rss_ik::SendIKCommand::ConstPtr &msg) {
        ROS_INFO("Processing IK Command");
        typedef rss_ik::SendIKCommand Com;
        double x = msg->x;
        double y = msg->y;
        while (false) {
            tf::StampedTransform transform;
            try {
                listener->lookupTransform(
                        "/map",
                        "/arm_base_link",
                        ros::Time(0),
                        transform);
                x -= transform.getOrigin().getX();
                y -= transform.getOrigin().getY();
                break;
            }
            catch (tf::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(0.5).sleep();
            }
        }
        std_msgs::Int16 status;
        status.data = 1;
        statusPub.publish(status);
        switch(msg->command) {
            case Com::BUTTON:
                ROS_INFO("BUTTON");
                FrameSequence(rss::sequences::BUTTON).play(x, y, jointPub, targetServiceClient);
                break;
            case Com::PUSH_RIGHT:
                ROS_INFO("PUSH_RIGHT");
                FrameSequence(rss::sequences::PUSH_RIGHT).play(x, y, jointPub, targetServiceClient);
                break;
            case Com::PUSH_LEFT:
                ROS_INFO("PUSH_LEFT");
                FrameSequence(rss::sequences::PUSH_LEFT).play(x, y, jointPub, targetServiceClient);
                break;
            case Com::SCOOP_RIGHT:
                FrameSequence(rss::sequences::SCOOP_RIGHT).play(x, y, jointPub, targetServiceClient);
                break;
            case Com::SCOOP_LEFT:
                FrameSequence(rss::sequences::SCOOP_LEFT).play(x, y, jointPub, targetServiceClient);
                break;
            case Com::SCOOP_CENTER:
                FrameSequence(rss::sequences::SCOOP_CENTER).play(x, y, jointPub, targetServiceClient);
                break;
            case Com::GRAB:
                FrameSequence(rss::sequences::GRAB).play(x, y, jointPub, targetServiceClient);
                break;
            case Com::LIFT:
                FrameSequence(rss::sequences::LIFT).play(x, y, jointPub, targetServiceClient);
                break;
            default:
                break;
        }
        status.data = 2;
        statusPub.publish(status);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ik_command_server");
    ros::NodeHandle n("~ik");
    rss::armPosSub = n.subscribe("/state_machine/arm_position", 1, rss::processCommand);
    rss::jointPub = n.advertise<std_msgs::Float64MultiArray>("/joint_trajectory_point", 1);
    rss::statusPub = n.advertise<std_msgs::Int16>("/arm_controller/status", 1, true);
    rss::targetServiceClient = n.serviceClient<rss_ik::GetTargetJointAngles>("/get_target_joint_angles", true);
    ROS_INFO("Starting IK Command Server");
    tf::TransformListener l;
    rss::listener = &l;
    std_msgs::Int16 status;
    status.data = 0;
    rss::statusPub.publish(status);
    ros::spin();
    return 0;
}
