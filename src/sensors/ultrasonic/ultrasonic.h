#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#define UTRASONIC_POWER_FRONT 48
#define UTRASONIC_POWER_SIDE 49

// Echo and trigger pins for ultrasonic sensor
#define TRIGGER_FRONT 30
#define ECHO_FRONT 31
#define TRIGGER_SIDE 10
#define ECHO_SIDE 11

#define ULTRASONIC_SAMPLING_RATE 25 // ms
#define SUDDEN_CHANGE_CLAMP 600
#define NOISE_CLAMP 10000

// Kalaman constants:
#define R 40
#define H 1

class Ultrasonic
{
public:
    char position;
    Ultrasonic(char pos);
    void init();
    int readDistance();
    double kalmanFilter(double U);
    bool firstReading = true;

private:
    unsigned long lastReadTime = 0;
    int prevDistReading;

    // Kalman vars
    double Q = 10;
    double P = 0;
    double U_hat = 0;
    double K = 0;
};

#endif