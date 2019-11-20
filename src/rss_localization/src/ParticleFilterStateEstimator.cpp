#include "rss_grid_localization/ParticleFilterStateEstimator.h"
#include <cmath>
#include <random>
#include <algorithm>

namespace rss {

    uniform_real_distribution<> ParticleFilterStateEstimator::uniformLinDist = uniform_real_distribution<>(0.0, 1.0);
    uniform_real_distribution<> ParticleFilterStateEstimator::uniformRotDist = uniform_real_distribution<>(-M_PI, M_PI);
    normal_distribution<> ParticleFilterStateEstimator::normalDist = normal_distribution<>(0.0, 0.05);

    random_device ParticleFilterStateEstimator::rd;
    default_random_engine ParticleFilterStateEstimator::gen{ParticleFilterStateEstimator::rd()};


    ParticleFilterStateEstimator::ParticleFilterStateEstimator(MeasurementModel *measurementModel,
                                                               MotionModel *motionModel, unsigned long particleCount)
            : particleCount(particleCount), measurementModel(measurementModel), motionModel(motionModel) {
        // Initialisation
    }

    void ParticleFilterStateEstimator::actionUpdate(Action action) {
        for (Particle &particle : particles) {
            particle.move(action);
        }

    }

    void ParticleFilterStateEstimator::measurementUpdate(const Measurement &z, const Map &m) {
        weights.clear();
        weights.reserve(particleCount);
        vector<double> tempWeights;
        tempWeights.reserve(particleCount);
        double total = 0.0;
        for (Particle &particle : particles) {
            double prob = particle.measurementProb(z, m);
            total += prob;
            tempWeights.push_back(prob);
        }
        // Normalise weights
        for (double &tempWeight : tempWeights) {
            tempWeight = tempWeight / total;
            weights.push_back(tempWeight);
        }
    }

    void ParticleFilterStateEstimator::particleUpdate() {
        ROS_DEBUG("Resampling");
        double beta = 0;
        unsigned long index = 0;
        double maxWeight = 0;
        for (const double &w : weights) {
            if (w > maxWeight)
                maxWeight = w;
        }
        vector<Particle> pTemp;
        pTemp.reserve(particleCount);
        for (unsigned long i = 0; i < particleCount; ++i) {
            beta = beta + 2.0 * maxWeight * uniformLinDist(gen);
            while (beta > weights[index]) {
                beta = beta - weights[index];
                ROS_DEBUG("BETA %f", beta);
                index = (index + 1) % particleCount;
            }
            pTemp.emplace_back(particles[index]);
        }
        for (unsigned long i = 0; i < particleCount; ++i) {
            particles[i] = pTemp[i];
        }
        /*
         * def resampling(particles, w):
                N = len(particles)
                beta=0
                index=0
                w_max= max(w)
                p_temp=[]
                for i in range(N):
                    beta= beta+2.0*w_max*np.random.rand()
                    while beta>w[index]:
                        beta = beta - w[index]
                        index=(index + 1) % N
                    selectedParticle = copy(particles[index])
                    p_temp.append(selectedParticle) # if beta<w[index], this indexed particle is selected
                return p_temp
         */
//        stochasticUniversalSampling();
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
//                ROS_INFO("BETA %f", beta);
                index = (index + 1) % particleCount;
            };
            tempParticles.push_back(particles[index]);
        }
        for (unsigned long i = 0; i < particleCount; ++i) {
            particles[i] = tempParticles[i];
        }
    }

    void ParticleFilterStateEstimator::initialiseParticles(const Map &map) {
        ROS_DEBUG("Initialising Particles (%lu)", particleCount);
        // clear old particles
        particles.clear();
        weights.clear();

        particles.reserve(particleCount);
        weights.reserve(particleCount);

        SimplePose init = {2.1, 0.65, 0.0};
        for (unsigned long i = 0; i < particleCount; ++i) {
            SimplePose offset = {normalDist(gen), normalDist(gen), 0};
            SimplePose randPose = init + offset;
            particles.emplace_back(measurementModel, motionModel, randPose);
        }
        return;

        vector<unsigned long> freeSpaceIndices(map.freeSpaceIndices);
        shuffle(freeSpaceIndices.begin(), freeSpaceIndices.end(), gen);
        unsigned long remainingEmptyCellCount = freeSpaceIndices.size();
        unsigned long next = 0;
        for (unsigned long i = 0; i < particleCount; ++i) {
            MapPoint mp = cellIndexToMapPoint(freeSpaceIndices[next], map);
            auto w = gridToWorldCoords(mp, map);
            SimplePose randomPose{w.getX(), w.getY(), uniformRotDist(gen)};
            particles.emplace_back(measurementModel, motionModel, randomPose);
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