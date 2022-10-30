#include "ultrasonic.h"
#include <Arduino.h>

Ultrasonic::Ultrasonic(char position)
{
    position = position;
}

void Ultrasonic::init()
{
    switch (position)
    {
    case 'f':
        pinMode(TRIGGER_FRONT, OUTPUT);
        pinMode(ECHO_FRONT, INPUT);
        digitalWrite(TRIGGER_FRONT, LOW);
        break;
    case 's':
        pinMode(TRIGGER_SIDE, OUTPUT);
        pinMode(ECHO_SIDE, INPUT);
        digitalWrite(TRIGGER_SIDE, LOW);
        break;
    }
}

int Ultrasonic::readDistance()
{
    double duration = 0;
    switch (position)
    {
    case 'f':
        digitalWrite(TRIGGER_FRONT, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_FRONT, LOW);
        duration = pulseIn(ECHO_FRONT, HIGH);
        break;
    case 's':
        digitalWrite(TRIGGER_SIDE, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_SIDE, LOW);
        duration = pulseIn(ECHO_SIDE, HIGH);
        break;
    }
    return (duration / 58.0 * 10);
}