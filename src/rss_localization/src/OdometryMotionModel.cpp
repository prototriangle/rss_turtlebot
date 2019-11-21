#include "rss_grid_localization/OdometryMotionModel.h"
#include "rss_grid_localization/util.h"
#include <cmath>

using namespace std;

namespace rss {
    SimplePose OdometryMotionModel::run(SimplePose currentPose, const Action &action) {
        static double transThresh = 0.001;
        static double rotThresh = 0.00;
        bool pr = abs(action.trans) < transThresh || abs(action.rot) < rotThresh;
        double s1 = pr ? 0 : normalDistributionTra(gen);
        double s2 = pr ? 0 : normalDistributionRot(gen);
        double theta = currentPose.theta + action.rot + s2;
        double trans = action.trans + s1;
        return {
                currentPose.x + trans * cos(theta),
                currentPose.y + trans * sin(theta),
                theta
        };

    }
}
