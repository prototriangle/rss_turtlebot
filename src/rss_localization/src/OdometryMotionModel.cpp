#include "rss_grid_localization/OdometryMotionModel.h"
#include "rss_grid_localization/util.h"
#include <cmath>

using namespace std;

namespace rss {
    SimplePose OdometryMotionModel::run(SimplePose currentPose, const Action &action) {
        static double transThresh = 0.005;
        static double rotThresh = 0.015;
        double s1 = action.trans < transThresh ? 0 : normalDistribution1(gen);
        double s2 = abs(action.rot) < rotThresh ? 0 : normalDistribution2(gen);
        double theta = currentPose.theta + action.rot + s2;
        double trans = action.trans + s1;
        return {
                currentPose.x + trans * cos(theta),
                currentPose.y + trans * sin(theta),
                theta
        };

    }
}
