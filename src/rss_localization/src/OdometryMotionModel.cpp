#include "rss_grid_localization/OdometryMotionModel.h"
#include "rss_grid_localization/util.h"
#include <cmath>

using namespace std;

namespace rss {
    SimplePose OdometryMotionModel::run(SimplePose currentPose, const Action &action) {
        return {
                currentPose.x + action.trans * costable_lookup(currentPose.theta + action.rot),
                currentPose.y + action.trans * sintable_lookup(currentPose.theta + action.rot),
                currentPose.theta + action.rot
        };

    }
}
