#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <cmath>
#include <vector>
#include <random>

using namespace std;
using namespace std_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;

namespace rss {

    class World {
        Point normalisedToWorldCoord(Point p) {
            return p;
        }

        Point worldToNormalisedCoord(Point p) {
            return p;
        }
    };

    struct Measurement {
        Header header;
        vector<float> ranges;
    };

    struct WeightedParticle {
        Pose pose;
        double weight;
    };

    class SimplePose {
    public:
        SimplePose(double x, double y, double theta) : x(x), y(y), theta(theta) {}

        double x;
        double y;
        double theta;

        SimplePose operator+(const SimplePose &pose) {
            return {x + pose.x, y + pose.y, theta + pose.theta};
        }

        void operator+=(const SimplePose &pose) {
            x += pose.x;
            y += pose.y;
            theta += pose.theta;
        }
    };

    struct Action {
        double rot;
        double tra;
        double motorPower;
    };

    class MeasurementModel {
    public:
        virtual double run(const Measurement &z, SimplePose x) = 0;

    };

    class LidarMeasurementModel : MeasurementModel {
    public:
        LidarMeasurementModel() {
            // init
            auto i = 1;
        }

        double run(const Measurement &z, SimplePose x) final {

        }
    };

    class MotionModel {
    public:
        virtual SimplePose run(SimplePose currentPose, const Action &action) = 0;
    };

    class OdometryMotionModel : MotionModel {

        random_device rd{};
        default_random_engine gen{rd()};
        double a1, a2, a3, a4;
    public:
        explicit OdometryMotionModel(
                double a1 = 0.25,
                double a2 = 0.25,
                double a3 = 0.26,
                double a4 = 0.25) : a1(a1), a2(a2), a3(a3), a4(a4) {
        }

        SimplePose run(SimplePose currentPose, const Action &action) final {
            double stddev_rot = a1 * abs(action.rot) + a2 * abs(action.tra);
            double delta_rot = normal_distribution<>{action.rot, stddev_rot}(gen);

            double stddev_tra = a3 * abs(action.tra) + a4 * abs(action.rot);
            double delta_tra = normal_distribution<>{action.tra + stddev_tra}(gen);

            return {
                    currentPose.x + delta_tra * cos(currentPose.theta + delta_rot),
                    currentPose.y + delta_tra * sin(currentPose.theta + delta_rot),
                    currentPose.theta + delta_rot
            };

        }
    };

    class ScanProcessor {
    public:
        static LaserScan currentScan;

        static Measurement getMeasurement() {
            return {currentScan.header, currentScan.ranges};
        }

        static void recCallback(const LaserScan::ConstPtr &msg) {
            currentScan = *msg;
        }
    };

    LaserScan ScanProcessor::currentScan = LaserScan();

    class MapHandler {
    public:
        static OccupancyGrid currentMap;

        static void recCallback(const OccupancyGrid::ConstPtr &msg) {
            currentMap = *msg;
        }
    };

    OccupancyGrid MapHandler::currentMap = OccupancyGrid();

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

        void move(const Action &action) {
            pose = motionModel->run(pose, action);
        }

        double measurementProb(const Measurement &z) {
            return measurementModel->run(z, pose);
        }
    };

    class ParticleFilterStateEstimator {
    private:
        unsigned long particleCount;
        vector<Particle> particles;
        vector<double> weights;

        random_device rd{};
        default_random_engine gen{rd()};
        uniform_real_distribution<> uniformLinDist = uniform_real_distribution<>(0.0, 1.0);
        uniform_real_distribution<> uniformRotDist = uniform_real_distribution<>(-M_PI, M_PI);
    public:
        ParticleFilterStateEstimator(MeasurementModel *measurementModel,
                                     MotionModel *motionModel,
                                     unsigned long particleCount)
                : particleCount(particleCount) {
            // Initialisation
            particles.reserve(particleCount);
            weights.reserve(particleCount);
            for (Particle &particle : particles) {
                SimplePose randomPose{uniformLinDist(gen), uniformLinDist(gen), uniformRotDist(gen)};
                particle = Particle(measurementModel, motionModel, randomPose);
            }
        }

        void actionUpdate(Action action) {
            for (Particle &particle : particles) {
                particle.move(action);
            }

        }

        void measurementUpdate(const Measurement &z) {
            vector<double> tempWeights(particleCount);
            double total = 0.0;
            for (Particle &particle : particles) {
                double prob = particle.measurementProb(z);
                total += prob;
                tempWeights.push_back(prob);
            }
            // Normalise weights
            for (double &weight : weights) {
                weight = weight / total;
            }
        }

        void particleUpdate() {
            stochasticUniversalSampling();
        }

        void stochasticUniversalSampling() {
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
    };


    Pose sample_motion_model(Action action, Pose pose) {

    }

    double measurement_model(const Measurement &measurement, Pose pose, OccupancyGrid &grid) {
        return {};
    }

    vector<Pose>
    mcl(vector<Pose> &particles, Action action, const Measurement &measurement, OccupancyGrid &map) {
        vector<WeightedParticle> weighted_particles;
        //
        for (Pose particle : particles) {
            Pose pose_estimate = sample_motion_model(action, particle);
            double weight = measurement_model(measurement, pose_estimate, map);
            WeightedParticle wp = {pose_estimate, weight};
            weighted_particles.push_back(wp);
        }

        vector<Pose> new_particles;
        for (WeightedParticle p : weighted_particles) {
            new_particles.push_back(p.pose);
        }

        return new_particles;
    }
}

using namespace rss;

int main(int argc, char **argv) {
    ros::init(argc, argv, "localization");
    ros::NodeHandle n;
    ros::Subscriber scanSub = n.subscribe("scan", 20, ScanProcessor::recCallback);
    ros::Subscriber initialMapSub = n.subscribe("initial_map", 20, MapHandler::recCallback);
    ros::Publisher mapPub = n.advertise<OccupancyGrid>("grid_map", 2);
    ros::Rate loopRate(5);

    OccupancyGrid occupancyGrid;

    occupancyGrid.header.frame_id = "scan";
    occupancyGrid.header.seq = 0;
    occupancyGrid.info.height = 500;
    occupancyGrid.info.width = 350;
    occupancyGrid.info.resolution = 0.01f;
    occupancyGrid.info.origin = Pose();
    while (ros::ok()) {
        occupancyGrid.header.stamp = ros::Time::now();
        ++occupancyGrid.header.seq;
        occupancyGrid.info.map_load_time = ros::Time();
        mapPub.publish(occupancyGrid);
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}