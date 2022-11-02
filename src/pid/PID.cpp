#include "PID.h"
#include <math.h>
#include <Arduino.h>

PID::PID(double min, double max, float Kp, float Kd, float Ki)
{
    this->max = max;
    this->min = min;
    setKp(Kp);
    setKd(Kd);
    setKi(Ki);
};

float PID::calculate(double setPoint, double measurement)
{
    double currentTime = micros();

    // Calculate error
    double error = setPoint - measurement;

    double rateError = (error - prevError) * (100000.0 / (currentTime - prevTime)); // 1000000 is for seconds can be changes for a nice KD

    double integralError = (error) * (currentTime - prevTime) / 1000000 + prevIntegralError;

    float correction = Kp * error + Kd * rateError; // Ki * integralError;

    prevError = error;
    prevTime = currentTime;
    if (correction < min)
    {
        correction = min;
    }
    else if (correction > max)
    {
        correction = max;
    }

    return correction;
}

void PID::setKp(float Kp)
{
    this->Kp = Kp;
}

void PID::setKd(float Kd)
{
    this->Kd = Kd;
}

void PID::setKi(float Ki)
{
    this->Ki = Ki;
}

float PID::getKp()
{
    return Kp;
}

float PID::getKd()
{
    return Kd;
}

float PID::getKi()
{
    return Ki;
}
