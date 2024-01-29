#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace PID
{
    class PIDController
    {
        private:
            double _dt;
            double _max;
            double _min;
            double _Kp;
            double _Kd;
            double _Ki;
            double _pre_error;
            double _integral;

        public:
            PIDController(double dt, double max, double min, double Kp, double Ki, double Kd);
            double calculate(double setpoint, double pv);       //returns manipulated variable given a setpoint and process value
            ~PIDController();
    };
}

#endif // PID_CONTROLLER_H