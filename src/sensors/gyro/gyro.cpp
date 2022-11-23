#include "gyro.h"

// The "static" IMU object that our ISR will access
Gyro *Gyro_Wrapper::primary = new Gyro;

bool Gyro::init()
{
    dataReady = false;

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
    yaw.push(0);
    pitch.push(0);
    roll.push(0);

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

    if (dataReady)
    {
        uint8_t fifoBuffer[64]; // FIFO storage buffer

        Quaternion q;        // [w, x, y, z]         quaternion container
        VectorFloat gravity; // [x, y, z]            gravity vector
        float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

        int fifoBufferStatus = mpu_.dmpGetCurrentFIFOPacket(fifoBuffer);

        if (fifoBufferStatus)
        {
            mpu_.dmpGetQuaternion(&q, fifoBuffer);
            mpu_.dmpGetGravity(&gravity, &q);
            mpu_.dmpGetYawPitchRoll(ypr, &q, &gravity);

            // Exponential Filtering and noise clamping

            if (abs(abs(ypr[0] * 180 / M_PI) - abs(yaw.last())) < GYRO_NOISE_CLAMP)
            {
                yaw.push((MPU_FILTER_ALPHA * (ypr[0] * 180 / M_PI) + (1 - MPU_FILTER_ALPHA) * yaw.last()));
            }

            if (abs(abs(ypr[1] * 180 / M_PI) - abs(pitch.last())) < GYRO_NOISE_CLAMP)
            {
                pitch.push((MPU_FILTER_ALPHA * (ypr[1] * 180 / M_PI) + (1 - MPU_FILTER_ALPHA) * pitch.last()));
            }

            if (abs(abs(ypr[2] * 180 / M_PI) - abs(roll.last())) < GYRO_NOISE_CLAMP)
            {
                roll.push((MPU_FILTER_ALPHA * (ypr[2] * 180 / M_PI) + (1 - MPU_FILTER_ALPHA) * roll.last()));
            }
        }
    }
    dataReady = false;
}

uint8_t Gyro::getIntStatus()
{
    return mpu_.getIntStatus();
}

float Gyro::getYaw()
{
    return yaw.last();
}

float Gyro::getPitch()
{
    return pitch.last();
}

float Gyro::getRoll()
{
    return roll.last();
}
