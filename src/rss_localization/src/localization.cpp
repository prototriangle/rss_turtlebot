#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/ColorRGBA.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Transform.h"
#include "rss_grid_localization/util.h"
#include "rss_grid_localization/ScanProcessor.h"
#include "rss_grid_localization/MapHandler.h"
#include "rss_grid_localization/ParticleFilterStateEstimator.h"
#include "rss_grid_localization/OdometryMotionModel.h"
#include "rss_grid_localization/LidarMeasurementModel.h"

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
bool firstAction = true;

void odomCallback(const Odometry::ConstPtr &msg) {
    newAction = true;
    lastReceivedOdom = *msg;
    if (firstAction) {
        lastUsedOdom = lastReceivedOdom;
        firstAction = false;
    }
}

bool newPoint = false;
PointStamped clicked;

void clickedPointCallback(const PointStamped::ConstPtr &msg) {
    clicked = *msg;
    newPoint = true;
}

bool newPose = false;
Pose rvizPose;
double rvizPoseVal;

void rvizPoseCallback(const PoseWithCovarianceStamped::ConstPtr &msg) {
    rvizPose = msg->pose.pose;
    newPose = true;
}

Action getAction() {
    newAction = false;
    ros::Duration stamp = lastReceivedOdom.header.stamp - lastUsedOdom.header.stamp;
    lastUsedOdom = lastReceivedOdom;
    return {lastReceivedOdom.twist.twist, stamp.toSec()};
}

bool shouldResample(const Action &action) {
    static const double trans_threshold = 0.0005;
    static const double rot_threshold = 0.04;
//    return true;
    return (abs(action.trans) > trans_threshold || abs(action.rot) > rot_threshold);
}

void
publishPoses(const ros::Publisher &posePub,
             const ros::Publisher &posesPub,
             const ros::Publisher &weightsPub,
             const ParticleFilterStateEstimator &pf,
             unsigned long &seq,
             tf::TransformBroadcaster &br,
             const Measurement &z
) {
    auto genericHeader = Header();
    genericHeader.frame_id = "map";
    genericHeader.seq = seq;

    double maxWeight = 0;
    unsigned long maxWeightIndex = 0;
    for (unsigned long i = 0; i < pf.weights.size(); ++i) {
        if (pf.weights[i] > maxWeight) {
            maxWeight = pf.weights[i];
            maxWeightIndex = i;
        }
    }

    Odometry poseEstimate;
    tf::Quaternion q;
    q.setRPY(0, 0, pf.particles[maxWeightIndex].pose.theta);
    ROS_DEBUG("THETA: %f", pf.particles[maxWeightIndex].pose.theta);
    poseEstimate.header = genericHeader;
    poseEstimate.pose.pose.position.x = pf.particles[maxWeightIndex].pose.x;
    poseEstimate.pose.pose.position.y = pf.particles[maxWeightIndex].pose.y;
    poseEstimate.pose.pose.position.z = 0;
    tf::quaternionTFToMsg(q, poseEstimate.pose.pose.orientation);
    posePub.publish(poseEstimate);

    if (posesPub.getNumSubscribers() > 0 && weightsPub.getNumSubscribers() > 0) {
        PoseArray currentPoses;
        currentPoses.poses.reserve(pf.particles.size());
        for (const auto &p : pf.particles) {
            tf::Quaternion q;
            q.setRPY(0, 0, p.pose.theta);
            Pose newPose;
            tf::quaternionTFToMsg(q, newPose.orientation);
            newPose.position.x = p.pose.x;
            newPose.position.y = p.pose.y;
            currentPoses.poses.push_back(newPose);
        }
        currentPoses.header = genericHeader;
        ROS_DEBUG("Publishing %lu particles", currentPoses.poses.size());
        posesPub.publish(currentPoses);

        MarkerArray currentWeights;
        currentWeights.markers.reserve(pf.particles.size());
        unsigned long i;
        for (i = 0; i < pf.particles.size(); ++i) {
            Marker marker;
            marker.header = currentPoses.header;
            marker.id = i;
            marker.pose = currentPoses.poses[i];
            marker.type = Marker::SPHERE;
            marker.ns = "Weights";
            geometry_msgs::Vector3 scale;
            scale.x = 0.025;
            scale.y = 0.025;
            scale.z = 0.025;
            marker.scale = scale;
            ColorRGBA c;
            c.r = maxWeight != 0.0 ? pf.weights[i] / maxWeight : 1.0;
            c.g = maxWeight != 0.0 ? pf.weights[i] / maxWeight : 1.0;
            c.b = maxWeight != 0.0 ? pf.weights[i] / maxWeight : 1.0;
            c.a = 1;
            marker.color = c;
            currentWeights.markers.push_back(marker);
        }
        if (newPose) {
            Marker marker;
            marker.header = genericHeader;
            marker.id = i;
            marker.pose = rvizPose;
            marker.type = Marker::ARROW;
            geometry_msgs::Vector3 scale;
            scale.x = 0.2;
            scale.y = 0.05;
            scale.z = 0.05;
            marker.scale = scale;
            ColorRGBA c;
            c.r = 1.0;
            c.g = 0.7;
            c.b = 0.0;
            c.a = 1;
            marker.color = c;
            currentWeights.markers.push_back(marker);

            marker.id = ++i;
            marker.type = Marker::TEXT_VIEW_FACING;
            double w = pf.measurementModel->run(z, marker.pose, MapHandler::currentMap);
            marker.text = to_string(w);
            marker.scale.z = 0.3;
            currentWeights.markers.push_back(marker);
            newPose = false;
            
            marker = Marker();
            marker.header = genericHeader;
            marker.type = Marker::LINE_LIST;
            marker.pose = poseEstimate.pose.pose;
            marker.scale.x = 0.01;
            marker.id = ++i;
            marker.ns = "Ranges";
            marker.color.a = 1.0;
            marker.color.r = 0.1;
            marker.color.g = 0.2;
            marker.color.b = 0.8;
            marker.points.clear();
            tf::Quaternion laserRot;
            laserRot.setRPY(0, 0, z.laserPose.theta);
            tf::quaternionTFToMsg(laserRot, marker.pose.orientation);
            marker.pose.position.x = z.laserPose.x;
            marker.pose.position.y = z.laserPose.y;
            Point p0;
            p0.x = 0;
            p0.y = 0;
            for (const auto &z_k : z.data) {
                marker.points.push_back(p0);
                Point p1;
                double theta = z_k.angle;
                double hitX = z_k.range * cos(theta);
                double hitY = z_k.range * sin(theta);
                p1.x = hitX;
                p1.y = hitY;
                marker.points.push_back(p1);
            }
            currentWeights.markers.push_back(marker);

            weightsPub.publish(currentWeights);
        }
    }

    seq = seq + 1;

    tf::Transform transform;
    transform.setOrigin(
            tf::Vector3(
                    poseEstimate.pose.pose.position.x,
                    poseEstimate.pose.pose.position.y,
                    poseEstimate.pose.pose.position.z));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_footprint"));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "localization");
    ros::NodeHandle n("~");
    ROS_INFO("STARTING LOCALIZATION...");
    ScanProcessor scanProcessor(360);
    ros::Subscriber scanSub = n.subscribe("/scan", 20, &ScanProcessor::recCallback, &scanProcessor);
    ros::Subscriber initialMapSub = n.subscribe("/lf", 20, MapHandler::recCallback);
    ros::Subscriber odomSub = n.subscribe("/odom", 20, odomCallback);
    ros::Publisher posesPub = n.advertise<PoseArray>("/particles", 2);
    ros::Publisher posePub = n.advertise<Odometry>("/pose_estimate", 2);
    ros::Publisher weightsPub = n.advertise<MarkerArray>("/weights", 2);
    ros::Subscriber clickedSub = n.subscribe("/clicked_point", 2, clickedPointCallback);
    ros::Subscriber rvizPoseSub = n.subscribe("/rviz_pose", 2, rvizPoseCallback);
    ros::Publisher debugMarkers = n.advertise<PointStamped>("/debug_markers", 2);
    ros::Rate loopRate(20);

    tf::TransformBroadcaster br;

    double particleCountD;
    bool pc = n.param<double>("particle_count", particleCountD, 64.0);
    auto particleCount = (unsigned long) abs(particleCountD);

    double sigmaHit;
    bool sh = n.param<double>("sigma_hit", sigmaHit, 0.2);
    double lambdaShort;
    bool ls = n.param<double>("lambda_short", lambdaShort, 1.0);
    LidarMeasurementModel measurementModel(0.94, 0.03, 0.01, 0.02, sigmaHit, lambdaShort);

    double sigmaRot;
    bool sr = n.param<double>("sigma_rot", sigmaRot, 0.014);
    double sigmaTra;
    bool st = n.param<double>("sigma_tra", sigmaTra, 0.005);
    OdometryMotionModel motionModel(sigmaRot, sigmaTra);

    ParticleFilterStateEstimator pf = ParticleFilterStateEstimator(&measurementModel, &motionModel, particleCount);

    // Warnings
    if (!pc)
        ROS_WARN("~particle_count not defined, using default");
    if (!sh)
        ROS_WARN("~sigma_hit not defined, using default");
    if (!ls)
        ROS_WARN("~lambda_short not defined, using default");
    if (!sr)
        ROS_WARN("~sigma_rot not defined, using default");
    if (!st)
        ROS_WARN("~sigma_tra not defined, using default");

    unsigned long seq = 0;
    ROS_INFO("Waiting for valid map");
    while (ros::ok()) {
        ros::spinOnce();
        if (MapHandler::currentMap.valid)
            break;
        loopRate.sleep();
    }
    ROS_INFO("Done");

    ROS_INFO("Placing");
    pf.initialiseParticles(MapHandler::currentMap);
    publishPoses(posePub, posesPub, weightsPub, pf, seq, br, Measurement());

    ROS_INFO("STARTING MCL LOOP");
    while (ros::ok()) {
        ros::spinOnce();
        if (newPose) {
            pf.initialiseParticles(MapHandler::currentMap, SimplePose(rvizPose));
            newPose = false;
        }
        if (MapHandler::currentMap.valid && newAction) {
            ROS_DEBUG("New Action");
            Action action = getAction();
            ROS_DEBUG("Rotation: %f", action.rot);
            ROS_DEBUG("Translation: %f", action.trans);

            ROS_DEBUG("Filter has %lu particles", pf.particles.size());
            pf.actionUpdate(action);

            auto z = scanProcessor.getMeasurement();
            pf.measurementUpdate(z, MapHandler::currentMap);
//            publishPoses(posePub, posesPub, weightsPub, pf, seq, br);

            if (rand() < RAND_MAX / 4.0 && shouldResample(action)) {
                pf.particleUpdate();
            }
            publishPoses(posePub, posesPub, weightsPub, pf, seq, br, z);

        }
        loopRate.sleep();
    }

    return 0;
}