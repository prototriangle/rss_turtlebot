#include "rss_grid_localization/ScanProcessor.h"
#include "rss_grid_localization/util.h"
#include "sensor_msgs/LaserScan.h"

using namespace sensor_msgs;
using namespace std;

namespace rss {

    Measurement ScanProcessor::getMeasurement() {
        vector<RangeAnglePair> ranges;
        ranges.reserve(rayCount);
        for (unsigned int i = 0; i < rayCount; ++i) {
            if (!useBadDataValue || currentScan.ranges[i] != badDataValue) {
                if (currentScan.ranges[i] > currentScan.range_min
                    && currentScan.ranges[i] < currentScan.range_max) {
                    ranges.push_back({currentScan.ranges[i], angles[i]});
                }
            }
        }
        return {currentScan.header, ranges};
    }

    void ScanProcessor::recCallback(const LaserScan::ConstPtr &msg) {
        currentScan = *msg;
    }

    ScanProcessor::ScanProcessor(unsigned int rayCount) : rayCount(rayCount) {
        angles.reserve(rayCount);
        if (rayCount == 0) {
            // No rays default to 360 rays
            this->rayCount = defaultRayCount;
        }
        double degIncr = 360.0 / this->rayCount;
        for (double currAngle = 0; currAngle < 360; currAngle += degIncr)
            angles.push_back(currAngle);
    }

    ScanProcessor::ScanProcessor(unsigned int rayCount, float badDataValue) : ScanProcessor(rayCount) {
        this->badDataValue = badDataValue;
        this->useBadDataValue = true;
    }

}
