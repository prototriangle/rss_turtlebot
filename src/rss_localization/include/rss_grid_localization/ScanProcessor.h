#ifndef RSS_LOCALIZATION_SCANPROCESSOR_H
#define RSS_LOCALIZATION_SCANPROCESSOR_H

#include "util.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>

using namespace sensor_msgs;


namespace rss {

    class ScanProcessor {
    private:
        unsigned int rayCount = 0;
        vector<double> angles;
    public:
        ScanProcessor(unsigned int rayCount);

        LaserScan currentScan;

        Measurement getMeasurement();

        void recCallback(const LaserScan::ConstPtr &msg);
    };

}

#endif //RSS_LOCALIZATION_SCANPROCESSOR_H
