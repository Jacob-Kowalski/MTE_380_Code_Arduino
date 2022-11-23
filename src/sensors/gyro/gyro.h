#ifndef GYRO_H
#define GYRO_H

#include <MPU6050_6Axis_MotionApps20.h>

#define GYRO_SAMPLING_RATE 10 // ms

#define GYRO_INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards

#include <CircularBuffer.h>

// Simple exponential filter to reduce noise, see https://en.wikipedia.org/wiki/Exponential_smoothing
// 0 < ALPHA <= 1, lower alpha means more smoothing
#define MPU_FILTER_ALPHA 0.8

// Number of values of history to keep
#define MPU_BUFFER_LEN 10

#define GYRO_NOISE_CLAMP 90

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
    unsigned long lastReadTime = 0;
    volatile bool dataReady;
    CircularBuffer<int16_t, MPU_BUFFER_LEN> yaw;
    CircularBuffer<int16_t, MPU_BUFFER_LEN> pitch;
    CircularBuffer<int16_t, MPU_BUFFER_LEN> roll;
};

class Gyro_Wrapper
{
public:
    static Gyro *primary;
    static void onDataReady();
};

#endif