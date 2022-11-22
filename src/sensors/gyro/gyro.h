#ifndef GYRO_H
#define GYRO_H

#include <MPU6050_6Axis_MotionApps20.h>

#define GYRO_SAMPLING_RATE 30 // ms

#define GYRO_INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards

// This way we can refer to `imu` in our application code
#define mpu Gyro_Wrapper::primary

// We use a seperate wrapper class with a static member
// so that we can access the IMU from within an interrupt
// service routine
class Gyro_Wrapper;

class Gyro
{
    friend class Gyro_Wrapper;

public:
    bool init();
    void readData();
    float getYaw();
    float getPitch();
    float getRoll();
    uint8_t getIntStatus();
    void run();

private:
    MPU6050 mpu_;
    uint16_t imu_packetsize_;
    float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    unsigned long lastReadTime = 0;
    volatile bool dataReady;
};

class Gyro_Wrapper
{
public:
    static Gyro *primary;
    static void onDataReady();
};

#endif