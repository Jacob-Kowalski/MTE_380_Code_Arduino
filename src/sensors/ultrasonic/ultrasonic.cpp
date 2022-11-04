#include "ultrasonic.h"
#include <Arduino.h>

Ultrasonic::Ultrasonic(char pos)
{
    position = pos;
}

void Ultrasonic::init()
{
    switch (position)
    {
    case 'f':
        pinMode(UTRASONIC_POWER_FRONT, OUTPUT);
        delay(100);
        pinMode(TRIGGER_FRONT, OUTPUT);
        pinMode(ECHO_FRONT, INPUT);
        digitalWrite(TRIGGER_FRONT, LOW);
        digitalWrite(UTRASONIC_POWER_FRONT, HIGH);
        break;
    case 's':
        pinMode(UTRASONIC_POWER_SIDE, OUTPUT);
        delay(100);
        pinMode(TRIGGER_SIDE, OUTPUT);
        pinMode(ECHO_SIDE, INPUT);
        digitalWrite(TRIGGER_SIDE, LOW);
        digitalWrite(UTRASONIC_POWER_SIDE, HIGH);
        break;
    }
}

int Ultrasonic::readDistance()
{
    if (millis() - lastReadTime <= SAMPLING_RATE)
    {
        delay(SAMPLING_RATE - (millis() - lastReadTime));
    }
    double distance = 0;
    switch (position)
    {
    case 'f':
        digitalWrite(TRIGGER_FRONT, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_FRONT, LOW);
        distance = pulseIn(ECHO_FRONT, HIGH) / 58.0 * 10;
        break;
    case 's':
        digitalWrite(TRIGGER_SIDE, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_SIDE, LOW);
        distance = pulseIn(ECHO_SIDE, HIGH) / 58.0 * 10;
        break;
    }

    // Clamp Sensor noise
    if (distance > NOISE_CLAMP)
    {
        distance = prevDistReading;
    }

    // Clamp sudden changes that are caused by sensor noise
    // if (abs(distance - prevDistReading) > SUDDEN_CHANGE_CLAMP && !firstReading)
    // {
    //     distance = prevDistReading;
    // }

    firstReading = false;
    prevDistReading = distance;

    // The below code outputs the filtered code.
    distance = kalmanFilter(distance);

    return distance;
}

// The below is an implementation for the kalman filter for side sensor
double Ultrasonic::kalmanFilter(double U)
{
    K = P * H / (H * P * H + R);
    U_hat += +K * (U - H * U_hat);
    P = (1 - K * H) * P + Q;
    return U_hat;
}