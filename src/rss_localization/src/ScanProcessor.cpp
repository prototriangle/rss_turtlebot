#include "ScanProcessor.h"
#include "util.h"
#include "sensor_msgs/LaserScan.h"

using namespace sensor_msgs;
using namespace std;

namespace rss {

    Measurement ScanProcessor::getMeasurement() {
        vector<RangeAnglePair> ranges;
        ranges.reserve(rayCount);
        for (unsigned int i = 0; i < rayCount; ++i) {
            ranges.push_back({currentScan.ranges[i], angles[i]});
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
            rayCount = 360;
            this->rayCount = rayCount;
        }
        double degIncr = 360.0 / rayCount;
        for (double currAngle = 0; currAngle < 360; currAngle += degIncr)
            angles.push_back(currAngle);
    }

}
