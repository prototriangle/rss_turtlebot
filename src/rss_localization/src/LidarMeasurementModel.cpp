#include "rss_grid_localization/util.h"
#include "rss_grid_localization/LidarMeasurementModel.h"

namespace rss {

double LidarMeasurementModel::run(const Measurement &z, const SimplePose &pose, const Map &map) {
  static bool lf = true;
  double q = 1.0;
  if (lf) {
    for (const RangeAnglePair &z_k : z.data) {
      if (z_k.range < max_range) {
        double theta = z_k.angle + pose.theta + z.laserPose.theta;
        double hitX = z_k.range * cos(theta) + z.laserPose.x;
        double hitY = z_k.range * sin(theta) + z.laserPose.y;
        MapPoint hitPoint = worldToGridCoords(pose.x + hitX, pose.y + hitY, map);
        double distance = likelihoodLookup(hitPoint, map);
        double n = distance;
        double m = p_rand(z_k.range);
        double nlprob = negLogProb(0.95 * n + 0.05 * m);
        q += nlprob;
      }
    }
    return q;
  } else {
    auto mp = worldToGridCoords(pose.x, pose.y, map);
    unsigned long hi = map.grid.data.size();
    unsigned long i = mp.x + mp.y * map.grid.info.width;
    bool oob = hi < i;
    if (oob) {
      i = hi;
    }
    if (oob || map.grid.data[i] > 50) {//occupied
      return 0.001;
    }
    for (RangeAnglePair z_k : z.data) {
      double pred = compute_noise_free_range(z_k.angle, pose, map);
      auto pHit = p_hit(z_k.range, pred);
      auto pShort = p_short(z_k.range, pred);
      auto pMax = p_max(z_k.range);
      auto pRand = p_rand(z_k.range);
      double p =
          z_hit * pHit
              + z_short * pShort
              + z_max * pMax
              + z_rand * pRand;
      q *= p;
    }
    return q;
  }
}

double
LidarMeasurementModel::compute_noise_free_range(const double &angle, const SimplePose &pose, const Map &map) {
  static const unsigned int t_max = 1024;
  double theta = angle + pose.theta;
  double ray_x = cos(theta);
  double ray_y = sin(theta);
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

LidarMeasurementModel::LidarMeasurementModel(double z_hit, double z_short, double z_max, double z_rand,
                                             double sigmaHit, double lamdaShort)
    : z_hit(z_hit), z_short(z_short), z_max(z_max), z_rand(z_rand), sigmaHit(sigmaHit),
      lambdaShort(lambdaShort) {
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
    return normal_pdf(range, prediction, sigmaHit);
  } else {
    return 0.0;
  }
}

double LidarMeasurementModel::p_short(const double &range, const double &prediction) {
  if (range >= 0.0 && range <= max_range) {
    double cumulative = 1 - exp(-lambdaShort * prediction);
    return lambdaShort * exp(-lambdaShort * range) / cumulative;
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

bool LidarMeasurementModel::withinMap(const MapPoint &p, const Map &map) {
  return p.x < map.grid.info.width && p.y < map.grid.info.height;
}

double LidarMeasurementModel::likelihoodLookup(MapPoint point, const Map &map) {
  if (withinMap(point, map)) {
    unsigned long i = point.x + point.y * map.grid.info.width;
    return (100.0 - (double) map.grid.data[i]) / 100.0;
  } else {
    return 1.0;
  }
}

}
