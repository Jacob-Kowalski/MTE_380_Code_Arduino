#include "gyro.h"

bool Gyro::init()
{
    uint8_t devStatus = mpu_.dmpInitialize();

    mpu_.CalibrateAccel(6);
    mpu_.CalibrateGyro(6);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        mpu_.setDMPEnabled(true);

        // get expected DMP packet size for later comparison
        imu_packetsize_ = mpu_.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.println("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.print(")");
        return false;
    }

    return true;
}

void Gyro::readData()
{
    // Restrict gyro sensor sampling rate
    if (millis() - lastReadTime <= GYRO_SAMPLING_RATE)
    {
        delay(GYRO_SAMPLING_RATE - (millis() - lastReadTime));
    }
    lastReadTime = millis();

    uint8_t fifoBuffer[64]; // FIFO storage buffer
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorFloat gravity;    // [x, y, z]            gravity vector

    if (mpu_.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
        mpu_.dmpGetQuaternion(&q, fifoBuffer);
        mpu_.dmpGetGravity(&gravity, &q);
        mpu_.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
}

float Gyro::getYaw()
{
    return ypr[0] * 180 / M_PI;
}

float Gyro::getPitch()
{
    return ypr[1] * 180 / M_PI;
}

float Gyro::getRoll()
{
    return ypr[2] * 180 / M_PI;
}
