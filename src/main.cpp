#include <Arduino.h>
#include <Wire.h>

#include "sensors/gyro/gyro.h"
#include "sensors/ultrasonic/ultrasonic.h"
#include "motors/motors.h"

#include "pid/PID.h"
// =====================================================
// ===               Global Constants                ===
// =====================================================

const int MOTOR_MAX_SPEED = 255;
const int MOTOR_MIN_SPEED = 115;
const int TILE_LENGTH = 300; // mm

// =====================================================
// ===               Global Variables                ===
// =====================================================

// all units in mm

int turns = 0;
bool doneCourse = false;

long gyroprevtime = 0;
long gyrocurtime = 0;

// the wanted distances from each wall in mm
int frontWallLimit = 170;
int sideWallLimit = 100;

// Current ultrasonic measurements
int sideDistance, frontDistance = 0;

bool inPitTrap = false;

int linearInertia = 110;
int rotationalInertia = 11; // degrees

// Initialize Sensor objects
Gyro mpu;
Ultrasonic ultrasonicFront('f');
Ultrasonic ultrasonicSide('s');

// Initialize Motor object
Motors motors;

// PID Controllers
PID courseCorrection(-140, 140, 6, 100, 0);
PID stopCorrection(MOTOR_MIN_SPEED, MOTOR_MAX_SPEED, 2.5, 0.5, 0);
PID turnCorrection(MOTOR_MIN_SPEED, MOTOR_MAX_SPEED, 5, 100, 0);

// Test number that dictates which test is run
int test = 1;

// ========================================================
// ===               Function Prototypes                ===
// ========================================================

void updateAngles();
void initiateTurn();
void turn();
void checkPitTrap();
void stabilizeKalmanFilter(int readCount);

// ========================================================
// ===                     Setup                        ===
// ========================================================

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  // Initialize motors/ultrasonic/gyro
  motors.init();
  ultrasonicFront.init();
  ultrasonicSide.init();
  mpu.init();

  stabilizeKalmanFilter(10);

  delay(2000);

  motors.adjust(MOTOR_MAX_SPEED, 0);
}

// ========================================================
// ===                     Loop                         ===
// ========================================================

void loop()
{
  // speed test
  while (test == 1)
  {
    motors.start();
    while (ultrasonicSide.readDistance() < 100)
    {
    }
    delay(4000);
    motors.stop();
    while (1)
    {
    }
  }

  // 10cm from wall
  while (test == 2)
  {
    frontDistance = ultrasonicFront.readDistance();
    if (frontDistance <= frontWallLimit + linearInertia) // frontWallLimit)
    {
      motors.stop();
      while (1)
      {
      }
    }
    else // if (error >= 5)
    {
      motors.adjust(stopCorrection.calculate(frontDistance, frontWallLimit + linearInertia), 0);
    }
  }

  // turn accuracy
  while (test == 3)
  {
    delay(1000);
    turn();
    motors.adjust(0, 0);

    while (1)
    {
    }
  }
}

// ========================================================
// ===              Function Definitions                ===
// ========================================================

void updateAngles()
{
  gyrocurtime = millis();
  if (gyrocurtime - gyroprevtime < 5)
    delay(5 - (gyrocurtime - gyroprevtime));
  mpu.readData();
  gyroprevtime = gyrocurtime;
}

void initiateTurn()
{
  // Adjust set Point and turning distance based on number of turns made
  // sideWallLimit = 100 + floor((turns - 1) / 4) * 300;
  // frontWallLimit = 200 + floor((turns + 1) / 4) * 300;

  if (turns == 2 || turns == 6)
  {
    frontWallLimit = TILE_LENGTH + frontWallLimit;
  }
  if (turns == 3 || turns == 7)
  {
    sideWallLimit = TILE_LENGTH + sideWallLimit;
  }
  if ((turns) == 10)
  {
    delay(150);        // Move forward before shutting off motors
    doneCourse = true; // Will end main loop
  }
  else
  {
    turn();
  }
}

void turn()
{
  double prevAngle = 0;
  double currAngle = mpu.getYaw();
  prevAngle = currAngle;
  double angleTurned = 0;

  turnCorrection.prevTime = millis();
  turnCorrection.prevError = 0;

  while (angleTurned < 90 - rotationalInertia - turns / 2) // Upper bound should be adjusted alongside turnCorrectionPID
  {
    motors.adjust(turnCorrection.calculate(90 - rotationalInertia - turns / 2, angleTurned), 510);
    updateAngles();
    currAngle = mpu.getYaw();
    angleTurned = angleTurned + abs(abs(currAngle) - abs(prevAngle));
    prevAngle = currAngle;
  }

  motors.adjust(0, 0);
  delay(1000);

  turns++;

  courseCorrection.prevError = 0;
  stopCorrection.prevError = 0;

  courseCorrection.prevTime = millis();
  stopCorrection.prevTime = millis();

  ultrasonicSide.firstReading = true;
  ultrasonicFront.firstReading = true;

  // Stop motors after completing turn
  stabilizeKalmanFilter(20);
  motors.adjust(MOTOR_MAX_SPEED, 0);
}

void checkPitTrap()
{
  double angle = mpu.getPitch();
  if (angle > 5 && !inPitTrap)
  {
    inPitTrap = true;
  }
  else if (angle < -4 && inPitTrap)
  {
    inPitTrap = false;
  }
}

void stabilizeKalmanFilter(int readCount)
{
  for (int i = 0; i < readCount; i++)
  {
    frontDistance = ultrasonicFront.readDistance();
    sideDistance = ultrasonicSide.readDistance();
    delay(25);
  }
}