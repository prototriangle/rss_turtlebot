#ifndef RSS_IK_SEQUENCES_H
#define RSS_IK_SEQUENCES_H

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "rss_ik/GetTargetJointAngles.h"
#include <vector>
#include <iostream>

using namespace std;

namespace rss {
    class FrameSequence {
    public:
        FrameSequence(const string &serializedSeq) {
            istringstream stream(serializedSeq);
            string buffer;
            const unsigned int lineLength = 6;
            double x, y, z, phi, gripper, duration;
            double *vars[lineLength] = {&x, &y, &z, &phi, &gripper, &duration};
            double d = 0;
            unsigned int i = 0;
            while (stream >> d) {
                *vars[i] = d;
//                ROS_INFO("Value: %f", d);
                i = (i + 1) % lineLength;
                if (i == 0) {
                    frames.emplace_back(x, y, z, phi, gripper, duration);
                }
            }
        }

        class Frame {
        public:
            const double x_diff;
            const double y_diff;
            const double z_diff;
            const double phi_diff;
            const double gripper_diff;
            const double duration;

            Frame(const double &x, const double &y, const double &z, const double &phi, const double &gripper,
                  const double &duration) : x_diff(x), y_diff(y), z_diff(z), phi_diff(phi), gripper_diff(gripper),
                                            duration(duration) {}

            rss_ik::GetTargetJointAngles::Request
            toRequest(const double &x, const double &y, const double &z, const double &phi,
                      const double &gripper) const {
                rss_ik::GetTargetJointAngles::Request r;
                r.x = x + x_diff;
                r.y = y + y_diff;
                r.z = z + z_diff;
                r.phi = phi + phi_diff;
                r.gripper = gripper + gripper_diff;
                return r;
            }
        };

        vector<Frame> frames;

        void play(const double &x, const double &y, const ros::Publisher &jointTrajPub,
                  ros::ServiceClient &IKServiceClient) {
            if (!IKServiceClient)
                return;
            double currentX = x;
            double currentY = y;
            double currentZ = 0;
            double currentPhi = 0;
            double currentGripper = 0;
            for (const auto &frame : frames) {
                rss_ik::GetTargetJointAngles::Response res;
                rss_ik::GetTargetJointAngles::Request req = frame.toRequest(
                        currentX, currentY, currentZ, currentPhi, currentGripper
                );
                currentX = req.x;
                currentY = req.y;
                currentZ = req.z;
                currentPhi = req.phi;
                currentGripper = req.gripper;
                bool success = IKServiceClient.call(req, res);
                if (success) {
                    ros::Duration(frame.duration).sleep();
                    jointTrajPub.publish(res.angles);
                } else {
                    ROS_WARN("Call (%f, %f, %f, %f, %f) to IK Service failed.",
                             req.x, req.y, req.z, req.phi, req.gripper);
                    ros::Duration(frame.duration).sleep();
                    continue;
                }

            }
        }
    };
    namespace sequences {
        //scoop center initial 0.18 0 -0.12 -1.2 0 0.1
        const auto SCOOP_CENTER = R"(
0       0       -0.12   -1.5    0       1
-0.01   0       0       0       0       1
-0.01   0       0       0       0       1
-0.01   0       0       0       0       1
-0.01   0       0       0       0       1
-0.01   0       0       0       0       1
-0.01   0       0       0       0       1
0       0       0.02    0       0       1
0       0       0.04    0       0       1
0.04    0       0.01    0       0       1)";

        /*TODO*/
        const auto SCOOP_RIGHT = R"(
0       0       -0.12   -1.5    0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
0       0       0.02    0       0       0.1
0       0       0.04    0       0       0.1
0.04    0       0.01    0       0       0.1)";

        /*TODO*/
        const auto SCOOP_LEFT = R"(
0       0       -0.12   -1.5    0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
0       0       0.02    0       0       0.1
0       0       0.04    0       0       0.1
0.04    0       0.01    0       0       0.1)";

        const auto PUSH_RIGHT = R"(
0       0       -0.02   -0.1    0       0.1
0       -0.01   0       0       0       0.1
0       -0.01   0       0       0       0.1
0       -0.01   0       0       0       0.1
0       -0.01   0       0       0       0.1
0       -0.01   0       0       0       0.1
0       -0.01   0       0       0       0.1
0       -0.01   0.01    0       0       0.1
0       -0.01   0.01    0       0       0.1
0       -0.01   0.01    0       0       0.1
0       -0.01   0       0       0       0.1
0       -0.01   0       0       0       0.1
0       -0.01   0       0       0       0.1
0       -0.01   0       0       0       0.1
0       -0.01   0.01    0       0       0.1
0       -0.01   0.01    0       0       0.1
0       -0.01   0.01    0       0       0.1
0       -0.01   0       0       0       0.1
0       -0.01   0       0       0       0.1
0       -0.01   0.01    0       0       0.1
0       -0.01   0.01    0       0       0.1
0       -0.01   0.01    0       0       0.1)";

        const auto PUSH_LEFT = R"(
0       0       -0.02   -0.1    0       0.1
0       0.01    0       0       0       0.1
0       0.01    0       0       0       0.1
0       0.01    0       0       0       0.1
0       0.01   0       0       0       0.1
0       0.01   0       0       0       0.1
0       0.01   0       0       0       0.1
0       0.01   0.01    0       0       0.1
0       0.01   0.01    0       0       0.1
0       0.01   0.01    0       0       0.1
0       0.01   0       0       0       0.1
0       0.01   0       0       0       0.1
0       0.01   0       0       0       0.1
0       0.01   0       0       0       0.1
0       0.01   0.01    0       0       0.1
0       0.01   0.01    0       0       0.1
0       0.01   0.01    0       0       0.1
0       0.01   0       0       0       0.1
0       0.01   0       0       0       0.1
0       0.01   0.01    0       0       0.1
0       0.01   0.01    0       0       0.1
0       0.01   0.01    0       0       0.1)";

        /*TODO*/
        const auto BUTTON = R"(
0       0       0.02   -0.5     0       0.1
0       0       -0.02    0       0       0.1
0       0       -0.02    0       0       0.1
0       0       -0.02    0       0       0.1
0       0       -0.02    0       0       0.1
0       0       -0.0     0       0       0.1
0       0       0.04     0       0       0.1
0       0       0.01    0       0       0.1)";

        /*TODO*/
        const auto GRAB = R"(
0       0       -0.12   -1.5    0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
0       0       0.02    0       0       0.1
0       0       0.04    0       0       0.1
0.04    0       0.01    0       0       0.1)";

        /*TODO*/
        const auto LIFT = R"(
0       0       -0.12   -1.5    0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
-0.01   0       0       0       0       0.1
0       0       0.02    0       0       0.1
0       0       0.04    0       0       0.1
0.04    0       0.01    0       0       0.1)";


    }
}

#endif //RSS_IK_SEQUENCES_H
