#define UTRASONIC_POWER 50

// Echo and trigger pins for ultrasonic sensor
#define TRIGGER_FRONT 12
#define ECHO_FRONT 13
#define TRIGGER_SIDE 10
#define ECHO_SIDE 11

class Ultrasonic
{
public:
    char position;
    Ultrasonic(char position);
    void init();
    int readDistance();
};
