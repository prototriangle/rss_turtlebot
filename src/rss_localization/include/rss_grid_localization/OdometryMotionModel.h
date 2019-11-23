#ifndef RSS_LOCALIZATION_ODOMETRYMOTIONMODEL_H
#define RSS_LOCALIZATION_ODOMETRYMOTIONMODEL_H

#include "util.h"
#include "MotionModel.h"
#include <random>

using namespace std;

namespace rss {

class OdometryMotionModel : public MotionModel {

  random_device rd{};
  default_random_engine gen{rd()};
  double a1, a2, a3, a4;
  const double sigmaRot = 0.014;
  const double sigmaTra = 0.01;
  normal_distribution<> normalDistributionTra{0.0, sigmaTra};
  normal_distribution<> normalDistributionRot{0.0, sigmaRot};
 public:
  explicit OdometryMotionModel(
      double sigmaRot,
      double sigmaTra) : sigmaRot(sigmaRot), sigmaTra(sigmaTra) {
  }

  SimplePose run(SimplePose currentPose, const Action &action) final;
};

}

#endif //RSS_LOCALIZATION_ODOMETRYMOTIONMODEL_H
