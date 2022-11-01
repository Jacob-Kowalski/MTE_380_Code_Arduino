#include <Arduino.h>
#include <Wire.h>

#include "sensors/gyro/gyro.h"
#include "sensors/ultrasonic/ultrasonic.h"
#include "motors/motors.h"

// =====================================================
// ===               Global Constants                ===
// =====================================================

const int MOTOR_MAX_SPEED = 255;
const int MOTOR_MIN_SPEED = 115;

// =====================================================
// ===               Global Variables                ===
// =====================================================

// all units in mm

int turns = 1;
bool doneCourse = false;
int maxSpeed = 255;

// the wanted distances from each wall in mm
int frontWallLimit = 250;
int sideWallLimit = 100;

// Current measurements
int sideDistance = 0;
int frontDistance = 0;
int previousSideDistance = 50;
bool firstReading = true;

double error = 0;
double previousError = 0;
double previousIntegralError;
double PIDCorrection = 0;
double correction = 0;
// times for PID
double currentTime = 0;
double previousTime = 0;

// for ensuring sampling rate
uint32_t frontSensorTime = 0;
uint32_t sideSensorTime = 0;
bool inPitTrap = false;

int angleCount = 0;

// Initialize Sensor objects
Gyro mpu;
Ultrasonic ultrasonicFront('f');
Ultrasonic ultrasonicSide('s');

// Initialize Motor object
Motors motors;

// ========================================================
// ===               Function Prototypes                ===
// ========================================================

double courseCorrectionPID();
double turnCorrectionPID(double, double, double);
double turnCorrectionPID();
int getSideDistance();
int getFrontDistance();
void updateAngles();
void initiateTurn();
void turn();
void checkPitTrap();

// ========================================================
// ===                     Setup                        ===
// ========================================================

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  }

  // Initialize motors/ultrasonic/gyro
  motors.init();
  ultrasonicFront.init();
  ultrasonicSide.init();
  mpu.init();

  delay(2000);

  motors.adjust(maxSpeed, 0);
}

// ========================================================
// ===                     Loop                         ===
// ========================================================

void loop()
{
  if (doneCourse)
  {
    motors.stop();
  }
  else
  {
    sideDistance = getSideDistance();
    frontDistance = getFrontDistance();
    updateAngles();

    checkPitTrap();

    if (frontDistance <= 200 && !inPitTrap) // frontWallLimit)
    {
      initiateTurn();
    }
    else // if (error >= 5)
    {
      motors.adjust(maxSpeed, courseCorrectionPID());
    }
  }
}

// ========================================================
// ===                Helper Functions                  ===
// ========================================================

double courseCorrectionPID()
{

  currentTime = micros();

  // error is negative when far away from wall
  error = double(sideWallLimit - sideDistance);
  // 1000000 is for seconds can be changes for a nice KD
  double rateError = (error - previousError) * 1000000.0 / (currentTime - previousTime);
  // double integralError = (error) * (currentTime - previousTime) / 1000000 + previousIntegralError;
  correction = KP * error + KD * rateError;

  previousError = error;
  previousTime = currentTime;
  // previousIntegralError = integralError;
  // if (correction > 0)
  // {
  //   correction = correction * 5;
  // }
  if (abs(correction) > 140)
  {
    correction = (correction > 0) ? 140 : -140;
  }

  return correction;
}

double turnCorrectionPID(float &angleTurned, double &previousError, double &previousTime)
{
  //  Tested values on hard plastic floor
  double KP = 5000 / 1000;
  double KD = 100000 / 1000;

  double currentTime = micros();

  double error = 90 - angleTurned;
  double rateError = (error - previousError) * 1000000.0 / (currentTime - previousTime); // 1000000 is for seconds can be changes for a nice KD
  // double integralError = (error) * (currentTime - previousTime) / 1000000 + previousIntegralError;
  correction = KP * error + KD * rateError;

  previousError = error;
  previousTime = currentTime;
  if (abs(correction) > MOTOR_MAX_SPEED)
  {
    correction = MOTOR_MAX_SPEED;
  }
  if (abs(correction) < MOTOR_MIN_SPEED)
  {
    correction = MOTOR_MIN_SPEED;
  }
  return correction;
}

  if (abs(correction) < 115)
  {
    correction = 115;
  }
  return correction;
}

int getSideDistance()
{
  // checking delay time for max sensor
  if (millis() - sideSensorTime <= 25)
  {
    delay(25 - (millis() - sideSensorTime));
  }
  previousSideDistance = sideDistance;
  sideSensorTime = millis();
  int distance = ultrasonicSide.readDistance();

  // If difference between readings is too high, dismiss reading
  if (abs(distance - sideDistance) > 600 && !firstReading)
  {
    distance = sideDistance;
  }
  firstReading = false;
  return distance;
}

int getFrontDistance()
{
  // checking delay time for max sensor
  if (millis() - frontSensorTime <= 25)
  {
    delay(25 - (millis() - frontSensorTime));
  }
  frontSensorTime = millis();
  int distance = ultrasonicFront.readDistance();
  return distance;
}

void updateAngles()
{
  mpu.readData();
}

void initiateTurn()
{
  // Adjust set Point and turning distance based on number of turns made
  sideWallLimit = 30 + floor((turns - 1) / 4) * 300;
  frontWallLimit = 100 + floor(turns / 4) * 300;
  turns++;
  // Serial.print(turns);
  if ((turns - 1) == 11)
  {
    // shut off motors
    doneCourse = true;
  }
  else
  {
    turn();
  }
}

void turn()
{
  double previousError = 0;
  double previousTime = 0;

  double prevAngle = 0;
  double currAngle = mpu.getYaw();
  double angleTurned = 0;

  while (angleTurned < 90) // Upper bound should be adjusted alongside turnCorrectionPID
  {
    motors.adjust(turnCorrectionPID(angleTurned, previousError, previousTime), 510);
    updateAngles();
    currAngle = mpu.getYaw();
    angleTurned += abs(currAngle - prevAngle);
    prevAngle = currAngle;
  }
  // Stop motors after completing turn
  motors.adjust(0, 0);
}

void checkPitTrap()
{
  if (mpu.getPitch() < -10)
    inPitTrap = true;
  if (mpu.getPitch() > 10 && inPitTrap)
    inPitTrap = false;
}