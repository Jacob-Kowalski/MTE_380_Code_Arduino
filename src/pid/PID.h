#ifndef PID_H
#define PID_H

class PID
{
public:
    PID(double min, double max, float Kp, float Kd, float Ki);
    float calculate(double setPoint, double measurement);
    void setKp(float Kp);
    void setKd(float Kd);
    void setKi(float Ki);
    float getKp();
    float getKd();
    float getKi();
    double prevTime = 0, prevError = 0;

private:
    float Kp, Kd, Ki, prevIntegralError, max, min;
};

#endif