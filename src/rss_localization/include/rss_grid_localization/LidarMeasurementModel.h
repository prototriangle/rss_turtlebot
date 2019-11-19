#ifndef RSS_LOCALIZATION_LIDARMEASUREMENTMODEL_H
#define RSS_LOCALIZATION_LIDARMEASUREMENTMODEL_H

#include "MeasurementModel.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"

#include "util.h"
#include <cmath>
#include <vector>
#include <random>

using namespace std;
using namespace std_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;

namespace rss {
    class LidarMeasurementModel : public MeasurementModel {
        random_device rd{};
        default_random_engine gen{rd()};
        double a1{}, a2{}, a3{}, a4{};
        double sigma_hit = 0.2;
        double lambda_short = 1.0;
        double z_hit, z_short, z_max, z_rand;
        double max_range = 4.0;
    public:
        LidarMeasurementModel(double z_hit, double z_short, double z_max, double z_rand);

        double run(const Measurement &z, const SimplePose &pose, const Map &map) final;

    private:
        double compute_noise_free_range(const double &angle, const SimplePose &pose, const Map &map);

        double p_hit(const double &range, const double &prediction);

        double p_short(const double &range, const double &prediction);

        double p_max(const double &range);

        double p_rand(const double &range);

        static int checkMap(unsigned int x, unsigned int y, const Map &map);

        static bool withinMap(unsigned int x, unsigned int y, const Map &map);

        static bool withinMap(const MapPoint &p, const Map &map);

        double likelihoodLookup(MapPoint point, const Map &map);
    };
}

#endif //RSS_LOCALIZATION_LIDARMEASUREMENTMODEL_H
