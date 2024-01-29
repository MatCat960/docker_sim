#include <iostream>
#include <cmath>
#include "pid_control/PIDController.h"

namespace PID
{

    PIDController::PIDController(double dt, double max, double min, double Kp, double Ki, double Kd)
    {
        _dt = dt;
        _max = max;
        _min = min;
        _Kp = Kp;
        _Ki = Ki;
        _Kd = Kd;
        _pre_error = 0;
        _integral = 0;
    }

    double PIDController::calculate(double setpoint, double pv)
    {
        double error = setpoint - pv;
        double i = error * _dt;
        _integral += i;
        double derivative = (error - _pre_error) / _dt;
        // double output = _Kp * error + _Ki * _integral + _Kd * derivative;
        double output = 0.0;
        if (_Kp != 0.0) output += _Kp * error;
        if (_Ki != 0.0) output += _Ki * _integral;
        if (_Kd != 0.0) output += _Kd * derivative;
        
        if (output > _max)
            output = _max;
        else if (output < _min)
            output = _min;
        _pre_error = error;
        return output;
    }

    PIDController::~PIDController()
    {
    }



}