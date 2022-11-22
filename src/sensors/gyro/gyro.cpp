#include "gyro.h"

// The "static" IMU object that our ISR will access
Gyro *Gyro_Wrapper::primary = new Gyro;

bool Gyro::init()
{
    dataReady = false;

    // mpu_.initialize();

    pinMode(GYRO_INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu_.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // mpu_.CalibrateAccel(6);
    mpu_.CalibrateGyro(6);

    Serial.println(F("Initializing DMP..."));
    uint8_t devStatus = mpu_.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        Serial.println(F("Enabling DMP..."));
        mpu_.setDMPEnabled(true);

        // get expected DMP packet size for later comparison
        imu_packetsize_ = mpu_.dmpGetFIFOPacketSize();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
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

void Gyro::run()
{
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(GYRO_INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(GYRO_INTERRUPT_PIN), Gyro_Wrapper::onDataReady, RISING);
    Serial.println("IMU interrupt attached and ready!");
}

void Gyro_Wrapper::onDataReady()
{
    mpu->dataReady = true;
}

void Gyro::readData()
{
    // Restrict gyro sensor sampling rate
    if (millis() - lastReadTime <= GYRO_SAMPLING_RATE)
    {
        delay(GYRO_SAMPLING_RATE - (millis() - lastReadTime));
    }
    lastReadTime = millis();
    // Serial.println(dataReady);

    if (dataReady)
    {
        // uint16_t fifoCount = mpu_.getFIFOCount();
        // while (fifoCount < imu_packetsize_)
        // {
        //     fifoCount = mpu_.getFIFOCount();
        // }
        uint8_t fifoBuffer[64]; // FIFO storage buffer

        // mpu_.getFIFOBytes(fifoBuffer, imu_packetsize_);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        // fifoCount -= imu_packetsize_;

        Quaternion q;        // [w, x, y, z]         quaternion container
        VectorFloat gravity; // [x, y, z]            gravity vector

        int fifoBufferStatus = mpu_.dmpGetCurrentFIFOPacket(fifoBuffer);
        // if (fifoBufferStatus != 1)
        // {
        //     Serial.println("invalid fifo buffer status");
        // }
        // else if (fifoBufferStatus == 1)
        // {
        //     Serial.println("fifo buffer status is valid");
        // }
        if (fifoBufferStatus)
        {
            mpu_.dmpGetQuaternion(&q, fifoBuffer);
            mpu_.dmpGetGravity(&gravity, &q);
            mpu_.dmpGetYawPitchRoll(ypr, &q, &gravity);
        }

        // mpu_.resetFIFO();
    }
    dataReady = false;
}

uint8_t Gyro::getIntStatus()
{
    return mpu_.getIntStatus();
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
