#include "rss_grid_localization/util.h"
#include "rss_grid_localization/ScanProcessor.h"
#include "rss_grid_localization/MapHandler.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "rss_grid_localization/ParticleFilterStateEstimator.h"
#include "rss_grid_localization/OdometryMotionModel.h"
#include "rss_grid_localization/LidarMeasurementModel.h"
#include <vector>

using namespace std;
using namespace std_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;

using namespace rss;

Action lastAction;
bool newAction = false;

void actionCallback(const Twist::ConstPtr &msg) {
    tf2::Vector3 l;
    tf2::fromMsg(msg->linear, l);
    lastAction.trans = l.length();
    tf2::Vector3 a;
    tf2::fromMsg(msg->angular, a);
    lastAction.rot = a.getZ();
    newAction = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "localization");
    ros::NodeHandle n;
    ROS_INFO("STARTING LOCALIZATION...");
    auto scanProcessor = ScanProcessor(360);
    ros::Subscriber scanSub = n.subscribe("scan", 20, &ScanProcessor::recCallback, &scanProcessor);
    ros::Subscriber initialMapSub = n.subscribe("map", 20, MapHandler::recCallback);
    ros::Subscriber actionSub = n.subscribe("action", 20, actionCallback);
    ros::Publisher posesPub = n.advertise<PoseArray>("pose_estimate", 2);
    ros::Rate loopRate(5);

    unsigned long particleCount = 256;

    LidarMeasurementModel measurementModel(0.65, 0.2, 0.1, 0.05);
    OdometryMotionModel motionModel;
    ParticleFilterStateEstimator pf = ParticleFilterStateEstimator(&measurementModel, &motionModel, particleCount);

    ROS_INFO("STARTING MCL LOOP");
    PoseArray currentPoses;
    currentPoses.poses.reserve(particleCount);
    unsigned long seq = 0;
    while (ros::ok()) {
        if (MapHandler::currentMap.valid && newAction) {
            newAction = false;

            pf.initialiseParticles(MapHandler::currentMap);

            pf.actionUpdate(lastAction);

            pf.measurementUpdate(scanProcessor.getMeasurement(), MapHandler::currentMap);

            pf.particleUpdate();

            currentPoses.poses.clear();
            currentPoses.poses.reserve(particleCount);
            for (const auto &p : pf.particles) {
                Pose newPose;
                tf2::Quaternion q;
                q.setEuler(p.pose.theta, 0, 0);
                newPose.orientation = tf2::toMsg(q);
            }
            currentPoses.header = Header();
            currentPoses.header.frame_id = "map";
            currentPoses.header.seq = seq;
            posesPub.publish(currentPoses);
        }
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}