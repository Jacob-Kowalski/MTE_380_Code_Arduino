#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#define UTRASONIC_POWER_FRONT 48
#define UTRASONIC_POWER_SIDE 49

// Echo and trigger pins for ultrasonic sensor
#define TRIGGER_FRONT 12
#define ECHO_FRONT 13
#define TRIGGER_SIDE 10
#define ECHO_SIDE 11

class Ultrasonic
{
public:
    char position;
    Ultrasonic(char pos);
    void init();
    int readDistance();
};

#endif