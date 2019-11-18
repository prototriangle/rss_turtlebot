#include "rss_grid_localization/util.h"
#include "rss_grid_localization/LidarMeasurementModel.h"


namespace rss {

    double LidarMeasurementModel::run(const Measurement &z, const SimplePose &x, const Map &map) {
        double q = 1;
        for (RangeAnglePair z_k : z.data) {
            double pred = compute_noise_free_range(z_k.angle, x, map);
            double p =
                    z_hit * p_hit(z_k.range, pred)
                    + z_short * p_short(z_k.range, pred)
                    + z_max * p_max(z_k.range)
                    + z_rand * p_rand(z_k.range);
            q *= p;
        }
        return q;
    }

    double
    LidarMeasurementModel::compute_noise_free_range(const double &angle, const SimplePose &pose, const Map &map) {
        static const unsigned int t_max = 1024;
        double theta = rad2deg(angle + pose.theta);
        double ray_x = costable_lookup((unsigned int) theta);
        double ray_y = sintable_lookup((unsigned int) theta);
        unsigned int t = 0;
        int offset_x, offset_y;
        MapPoint origin = worldToGridCoords(pose.x, pose.y, map);
        bool hit = false;
        while (t < t_max) {
            offset_x = int(floor(ray_x * t + 0.5));
            offset_y = int(floor(ray_y * t + 0.5));
            unsigned int map_x = origin.x + offset_x;
            unsigned int map_y = origin.y + offset_y;
            if (!withinMap(map_x, map_y, map)) {
                break;
            }
            int val = checkMap(map_x, map_y, map);
            if (val > 50) {
                MapPoint hitPoint = {map_x, map_y};
                return distanceOnFromMap(origin, hitPoint, map);
            }
            ++t;
        }
        return z_max;
    }

    LidarMeasurementModel::LidarMeasurementModel(double z_hit, double z_short, double z_max, double z_rand)
            : z_hit(z_hit), z_short(z_short), z_max(z_max), z_rand(z_rand) {
        if (z_hit + z_short + z_max + z_rand != 1.0) {
            cout << "Beam model weights don't sum to 1.0" << endl;
            cout << "Setting to equal weights..." << endl;
            this->z_hit = 0.25;
            this->z_short = 0.25;
            this->z_max = 0.25;
            this->z_rand = 0.25;
        }
    }

    double LidarMeasurementModel::p_hit(const double &range, const double &prediction) {
        if (range >= 0.0 && range <= max_range) {
            return normal_pdf(range, prediction, sigma_hit);
        } else {
            return 0.0;
        }
    }

    double LidarMeasurementModel::p_short(const double &range, const double &prediction) {
        if (range >= 0.0 && range <= max_range) {
            double cumulative = 1 - exp(-lambda_short * prediction);
            return lambda_short * exp(-lambda_short * range) / cumulative;
        } else {
            return 0.0;
        }
    }

    double LidarMeasurementModel::p_max(const double &range) {
        if (range == max_range) {
            return 1.0;
        } else {
            return 0.0;
        }
    }

    double LidarMeasurementModel::p_rand(const double &range) {
        if (range >= 0.0 && range <= max_range) {
            return 1.0 / max_range;
        } else {
            return 0.0;
        }
    }

    int LidarMeasurementModel::checkMap(unsigned int x, unsigned int y, const Map &map) {
        return map.grid.data[x + y * map.grid.info.width];
    }

    bool LidarMeasurementModel::withinMap(unsigned int x, unsigned int y, const Map &map) {
        return x < map.grid.info.width && y < map.grid.info.height;
    }


}
