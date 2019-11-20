#ifndef RSS_LOCALIZATION_SCANPROCESSOR_H
#define RSS_LOCALIZATION_SCANPROCESSOR_H

#include "sensor_msgs/LaserScan.h"
#include "util.h"
#include <vector>

using namespace sensor_msgs;


namespace rss {

    class ScanProcessor {
    private:
        static const unsigned int defaultRayCount = 360;
        bool useBadDataValue = false;
        float badDataValue = 999.0f;
        unsigned int rayCount = 0;
        vector<double> angles;
    public:
        explicit ScanProcessor(unsigned int rayCount = defaultRayCount);
        ScanProcessor(unsigned int rayCount, float badDataValue);

        LaserScan currentScan;

        Measurement getMeasurement();

        void recCallback(const LaserScan::ConstPtr &msg);
    };

}

#endif //RSS_LOCALIZATION_SCANPROCESSOR_H
