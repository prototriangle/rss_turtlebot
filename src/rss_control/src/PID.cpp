//
// Created by ignat on 07/10/2019.
//

#include "PID.hpp"

PID::PID() {
    kp_ = 0;
    ki_ = 0;
    kd_ = 0;
    min_ = -5;
    max_ = 5;
}

void PID::setCoefs(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::setConstraints(double min, double max) {
    min_ = min;
    max_ = max;
}

double PID::compute(double error) {
    // first add to buffer
    if (error_buffer.size() > 10) {
        error_buffer.erase(error_buffer.begin());
    }
    error_buffer.push_back(error);

    // now compute controls
    auto p_part = kp_ * error;
    auto d_part = kd_ * fabs(error - error_buffer[error_buffer.back() - 1]);

    double error_sum;
    for (auto& n : error_buffer)
        error_sum += n;

    auto i_part = ki_ * error_sum;

    // sum all parts together
    double output = p_part + i_part + d_part;

    // apply constraints
    if (output > max_)
        output = max_;

    if (output < min_)
        output = min_;

    return output;
}
