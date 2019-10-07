//
// Created by ignat on 07/10/2019.
//

#ifndef RSS_CONTROL_PID_H
#define RSS_CONTROL_PID_H

#include <vector>
#include <math.h>


class PID {
public:
    PID();
    void setCoefs(double kp, double ki ,double kd);
    void setConstraints(double min, double max);
    double compute(double error);
private:
    double kp_, ki_,  kd_;
    double min_, max_;
    std::vector<double> error_buffer;
};


#endif //RSS_CONTROL_PID_H
