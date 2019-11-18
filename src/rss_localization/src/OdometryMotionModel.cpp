#include "rss_grid_localization/OdometryMotionModel.h"
#include "rss_grid_localization/util.h"
#include <cmath>

using namespace std;

namespace rss {
    SimplePose OdometryMotionModel::run(SimplePose currentPose, const Action &action) {
        double s1 = normalDistribution1(gen);
        double s2 = normalDistribution2(gen);
        return {
                currentPose.x + (action.trans + s1) * cos(currentPose.theta + action.rot),
                currentPose.y + (action.trans + s1) * sin(currentPose.theta + action.rot),
                currentPose.theta + action.rot + s2
        };

    }
}
