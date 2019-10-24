#ifndef RSS_LOCALIZATION_PARTICLEFILTERSTATEESTIMATOR_H
#define RSS_LOCALIZATION_PARTICLEFILTERSTATEESTIMATOR_H

#include "util.h"
#include "MeasurementModel.h"
#include "MotionModel.h"
#include <random>


namespace rss {

    class ParticleFilterStateEstimator {
    private:

        class Particle {
        private:
            MeasurementModel *measurementModel;
            MotionModel *motionModel;
        public:
            Particle(MeasurementModel *measurementModel,
                     MotionModel *motionModel,
                     SimplePose initialPose = {0, 0, 0})
                    : measurementModel(measurementModel), motionModel(motionModel), pose(initialPose) {
            }

            SimplePose pose;

            void move(const Action &action);

            double measurementProb(const Measurement &z, const Map &map);
        };

        unsigned long particleCount;
        vector<double> weights;

        static random_device rd;
        static default_random_engine gen;
        static uniform_real_distribution<> uniformLinDist;
        static uniform_real_distribution<> uniformRotDist;

        MeasurementModel *measurementModel;
        MotionModel *motionModel;

    public:
        vector<Particle> particles;

        ParticleFilterStateEstimator(MeasurementModel *measurementModel,
                                     MotionModel *motionModel,
                                     unsigned long particleCount);

        void actionUpdate(Action action);

        void measurementUpdate(const Measurement &z, const Map &m);

        void particleUpdate();

        void stochasticUniversalSampling();

        void initialiseParticles(const Map &map);
    };

}


#endif //RSS_LOCALIZATION_PARTICLEFILTERSTATEESTIMATOR_H
