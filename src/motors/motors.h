// Motor Pins
#define FRONT_LEFT_FORWARD 3
#define FRONT_LEFT_BACKWARD 2
#define BACK_LEFT_FORWARD 4
#define BACK_LEFT_BACKWARD 5
#define BACK_RIGHT_FORWARD 7
#define BACK_RIGHT_BACKWARD 6
#define FRONT_RIGHT_FORWARD 8
#define FRONT_RIGHT_BACKWARD 9

class Motors
{
public:
    void init();
    void start();
    void stop();
    void adjust(int maxSpeed, double correction);
};