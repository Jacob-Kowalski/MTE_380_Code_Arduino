#include <MPU6050_6Axis_MotionApps20.h>

class Gyro
{
public:
    bool init();
    void readData();
    float getYaw();
    float getPitch();
    float getRoll();

private:
    MPU6050 mpu_;
    uint16_t imu_packetsize_;
    float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
};
