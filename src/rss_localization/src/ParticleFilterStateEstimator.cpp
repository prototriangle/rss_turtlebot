#include "rss_grid_localization/ParticleFilterStateEstimator.h"
#include <cmath>
#include <random>
#include <algorithm>

namespace rss {

    uniform_real_distribution<> ParticleFilterStateEstimator::uniformLinDist = uniform_real_distribution<>(0.0, 1.0);
    uniform_real_distribution<> ParticleFilterStateEstimator::uniformRotDist = uniform_real_distribution<>(-M_PI, M_PI);

    random_device ParticleFilterStateEstimator::rd;
    default_random_engine ParticleFilterStateEstimator::gen{ParticleFilterStateEstimator::rd()};


    ParticleFilterStateEstimator::ParticleFilterStateEstimator(MeasurementModel *measurementModel,
                                                               MotionModel *motionModel, unsigned long particleCount)
            : particleCount(particleCount) {
        // Initialisation
    }

    void ParticleFilterStateEstimator::actionUpdate(Action action) {
        for (Particle &particle : particles) {
            particle.move(action);
        }

    }

    void ParticleFilterStateEstimator::measurementUpdate(const Measurement &z, const Map &m) {
        vector<double> tempWeights(particleCount);
        double total = 0.0;
        for (Particle &particle : particles) {
            double prob = particle.measurementProb(z, m);
            total += prob;
            tempWeights.push_back(prob);
        }
        // Normalise weights
        for (double &weight : weights) {
            weight = weight / total;
        }
    }

    void ParticleFilterStateEstimator::particleUpdate() {
        stochasticUniversalSampling();
    }

    void ParticleFilterStateEstimator::stochasticUniversalSampling() {
        double beta = uniformLinDist(gen) / double(particleCount);
        unsigned int index = 0;
        double increment = 1.0 / double(particleCount);
        vector<Particle> tempParticles;
        tempParticles.reserve(particleCount);
        for (Particle &particle : particles) {
            beta += increment;
            while (beta > weights[index]) {
                beta = beta - weights[index];
                index = (index + 1) % particleCount;
            };
            tempParticles.push_back(particle);
        }
        for (unsigned long i = 0; i < particleCount; ++i) {
            particles[i] = tempParticles[i];
        }
    }

    void ParticleFilterStateEstimator::initialiseParticles(const Map &map) {
        // clear old particles
        particles.clear();
        weights.clear();

        particles.reserve(particleCount);
        weights.reserve(particleCount);

        vector<unsigned long> freeSpaceIndices;
        for (unsigned long i = 0; i < map.grid.data.size(); i++) {
            if (map.grid.data[i] < 50) { // probably empty
                freeSpaceIndices.push_back(i);
            }
        }
        if (freeSpaceIndices.empty()) {
            ROS_WARN("No free space on map!");
        }
        shuffle(freeSpaceIndices.begin(), freeSpaceIndices.end(), gen);
        unsigned long remainingEmptyCellCount = freeSpaceIndices.size();
        unsigned long next = 0;
        for (Particle &particle : particles) {
            MapPoint mp = cellIndexToMapPoint(next, map);
            auto w = gridToWorldCoords(mp, map);
            SimplePose randomPose{w.getX(), w.getY(), uniformRotDist(gen)};
            particle = Particle(measurementModel, motionModel, randomPose);
            next = ((next + 1) % freeSpaceIndices.size());
            --remainingEmptyCellCount;
            if (remainingEmptyCellCount == 0) {
                shuffle(freeSpaceIndices.begin(), freeSpaceIndices.end(), gen);
                remainingEmptyCellCount = freeSpaceIndices.size();
            }
        }
    }

    void ParticleFilterStateEstimator::Particle::move(const Action &action) {
        pose = motionModel->run(pose, action);
    }

    double ParticleFilterStateEstimator::Particle::measurementProb(const Measurement &z, const Map &m) {
        return measurementModel->run(z, pose, m);
    }

}