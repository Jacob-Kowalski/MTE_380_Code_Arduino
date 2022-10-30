#include <Arduino.h>
#include <math.h>

#include "motors.h"

void Motors::init()
{
    pinMode(BACK_LEFT_FORWARD, OUTPUT);
    pinMode(BACK_LEFT_BACKWARD, OUTPUT);
    pinMode(FRONT_LEFT_FORWARD, OUTPUT);
    pinMode(FRONT_LEFT_BACKWARD, OUTPUT);
    pinMode(BACK_RIGHT_FORWARD, OUTPUT);
    pinMode(BACK_RIGHT_BACKWARD, OUTPUT);
    pinMode(FRONT_RIGHT_FORWARD, OUTPUT);
    pinMode(FRONT_RIGHT_BACKWARD, OUTPUT);
}

void Motors::start()
{
    digitalWrite(BACK_LEFT_FORWARD, HIGH);
    digitalWrite(BACK_LEFT_BACKWARD, LOW);
    digitalWrite(FRONT_LEFT_FORWARD, HIGH);
    digitalWrite(FRONT_LEFT_BACKWARD, LOW);
    digitalWrite(BACK_RIGHT_FORWARD, HIGH);
    digitalWrite(BACK_RIGHT_BACKWARD, LOW);
    digitalWrite(FRONT_RIGHT_FORWARD, HIGH);
    digitalWrite(FRONT_RIGHT_BACKWARD, LOW);
}

void Motors::stop()
{
    digitalWrite(BACK_LEFT_FORWARD, LOW);
    digitalWrite(BACK_LEFT_BACKWARD, LOW);
    digitalWrite(FRONT_LEFT_FORWARD, LOW);
    digitalWrite(FRONT_LEFT_BACKWARD, LOW);
    digitalWrite(BACK_RIGHT_FORWARD, LOW);
    digitalWrite(BACK_RIGHT_BACKWARD, LOW);
    digitalWrite(FRONT_RIGHT_FORWARD, LOW);
    digitalWrite(FRONT_RIGHT_BACKWARD, LOW);
}

void Motors::adjust(int maxSpeed, double correction)
{
    int16_t leftSpeed = ((correction > 0) ? maxSpeed : maxSpeed * (float)(1.0 - abs(correction) / 255.0));
    int16_t rightSpeed = ((correction < 0) ? maxSpeed : maxSpeed * (float)(1.0 - abs(correction) / 255.0));

    if (leftSpeed == 0)
        leftSpeed = 0;
    else if (abs(leftSpeed) < 115)
        leftSpeed = (leftSpeed > 0) ? 115 : -115;
    if (rightSpeed == 0)
        rightSpeed = 0;
    else if (abs(rightSpeed) < 115)
        rightSpeed = (rightSpeed > 0) ? 115 : -115;

    if (leftSpeed > 0)
    {
        analogWrite(BACK_LEFT_FORWARD, abs(leftSpeed));
        analogWrite(BACK_LEFT_BACKWARD, 0);
        analogWrite(FRONT_LEFT_FORWARD, abs(leftSpeed));
        analogWrite(FRONT_LEFT_BACKWARD, 0);
    }
    else
    {
        analogWrite(BACK_LEFT_FORWARD, 0);
        analogWrite(BACK_LEFT_BACKWARD, abs(leftSpeed));
        analogWrite(FRONT_LEFT_FORWARD, 0);
        analogWrite(FRONT_LEFT_BACKWARD, abs(leftSpeed));
    }

    if (rightSpeed > 0)
    {
        analogWrite(BACK_RIGHT_FORWARD, abs(rightSpeed));
        analogWrite(BACK_RIGHT_BACKWARD, 0);
        analogWrite(FRONT_RIGHT_FORWARD, abs(rightSpeed));
        analogWrite(FRONT_RIGHT_BACKWARD, 0);
    }
    else
    {
        analogWrite(BACK_RIGHT_FORWARD, 0);
        analogWrite(BACK_RIGHT_BACKWARD, abs(rightSpeed));
        analogWrite(FRONT_RIGHT_FORWARD, 0);
        analogWrite(FRONT_RIGHT_BACKWARD, abs(rightSpeed));
    }
}
