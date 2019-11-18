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
        const double sigma1 = 0.002;
        const double sigma2 = 0.01;
        normal_distribution<> normalDistribution1{0.0, sigma1};
        normal_distribution<> normalDistribution2{0.0, sigma2};
    public:
        explicit OdometryMotionModel(
                double a1 = 0.25,
                double a2 = 0.25,
                double a3 = 0.25,
                double a4 = 0.25) : a1(a1), a2(a2), a3(a3), a4(a4) {
        }

        SimplePose run(SimplePose currentPose, const Action &action) final;
    };

}


#endif //RSS_LOCALIZATION_ODOMETRYMOTIONMODEL_H
