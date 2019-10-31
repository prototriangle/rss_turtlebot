#ifndef RSS_LOCALIZATION_MOTIONMODEL_H
#define RSS_LOCALIZATION_MOTIONMODEL_H

#include "util.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <cmath>
#include <vector>
#include <random>

using namespace std;
using namespace std_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;

namespace rss {
    class MotionModel {
    public:
        virtual SimplePose run(SimplePose currentPose, const Action &action) = 0;
    };
}


#endif //RSS_LOCALIZATION_MOTIONMODEL_H
