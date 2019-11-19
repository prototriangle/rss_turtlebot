#include "rss_grid_localization/util.h"
#include "rss_grid_localization/ScanProcessor.h"
#include "rss_grid_localization/MapHandler.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/ColorRGBA.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/TransformStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Transform.h"
#include "rss_grid_localization/ParticleFilterStateEstimator.h"
#include "rss_grid_localization/OdometryMotionModel.h"
#include "rss_grid_localization/LidarMeasurementModel.h"
#include <vector>

using namespace std;
using namespace std_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace visualization_msgs;

using namespace rss;

Odometry lastUsedOdom;
Odometry lastReceivedOdom;
bool newAction = false;

void odomCallback(const Odometry::ConstPtr &msg) {
    newAction = true;
    lastReceivedOdom = *msg;
}

Action getAction() {
    newAction = false;
    ros::Duration stamp = lastReceivedOdom.header.stamp - lastUsedOdom.header.stamp;
    lastUsedOdom = lastReceivedOdom;
    return {lastReceivedOdom.twist.twist, stamp.toSec()};
}

bool shouldResample(const Action &action) {
    static const double trans_threshold = 0.0005;
    static const double rot_threshold = 0.0001;
//    return true;
    return (action.trans > trans_threshold && action.rot > rot_threshold);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "localization");
    ros::NodeHandle n;
    ROS_INFO("STARTING LOCALIZATION...");
    auto scanProcessor = ScanProcessor(360);
    ros::Subscriber scanSub = n.subscribe("scan", 20, &ScanProcessor::recCallback, &scanProcessor);
    ros::Subscriber initialMapSub = n.subscribe("map", 20, MapHandler::recCallback);
    ros::Subscriber odomSub = n.subscribe("odom", 20, odomCallback);
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
            ROS_INFO("New Action");
            Action action = getAction();

            pf.initialiseParticles(MapHandler::currentMap);

            pf.actionUpdate(action);

            pf.measurementUpdate(scanProcessor.getMeasurement(), MapHandler::currentMap);

            if (shouldResample(action))
                pf.particleUpdate();

            currentPoses.poses.clear();
            currentPoses.poses.reserve(particleCount);
            for (const auto &p : pf.particles) {
                Pose newPose;
                tf2::Quaternion q;
                q.setEuler(p.pose.theta, 0, 0);
                newPose.orientation = tf2::toMsg(q);
                newPose.position.x = p.pose.x;
                newPose.position.y = p.pose.y;
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