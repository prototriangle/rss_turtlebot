#ifndef RSS_LOCALIZATION_MEASUREMENTMODEL_H
#define RSS_LOCALIZATION_MEASUREMENTMODEL_H

#include "util.h"

namespace rss {

class MeasurementModel {
 public:
  virtual double run(const Measurement &z, const SimplePose &x, const Map &map) = 0;
};

}

#endif //RSS_LOCALIZATION_MEASUREMENTMODEL_H
