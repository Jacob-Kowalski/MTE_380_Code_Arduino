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

// =====================================================
// ===               Global Variables                ===
// =====================================================

// all units in mm

int turns = 1;
bool doneCourse = false;

// the wanted distances from each wall in mm
int frontWallLimit = 100;
int sideWallLimit = 100;

// Current ultrasonic measurements
int sideDistance = 0;
int frontDistance = 0;

bool inPitTrap = false;

int linearInertia = 80;
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

// ========================================================
// ===               Function Prototypes                ===
// ========================================================

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
    ;

  // Initialize motors/ultrasonic/gyro
  motors.init();
  ultrasonicFront.init();
  ultrasonicSide.init();
  mpu.init();

  delay(2000);

  motors.adjust(MOTOR_MAX_SPEED, 0);
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
    sideDistance = ultrasonicSide.readDistance();
    frontDistance = ultrasonicFront.readDistance();
    updateAngles();
    checkPitTrap();

    if (frontDistance <= frontWallLimit + linearInertia && !inPitTrap) // frontWallLimit)
    {
      initiateTurn();
    }
    else // if (error >= 5)
    {
      motors.adjust(stopCorrection.calculate(frontDistance, frontWallLimit + linearInertia), courseCorrection.calculate(sideWallLimit, sideDistance));
      // motors.adjust(255, courseCorrection.calculate(sideWallLimit, sideDistance));
    }
  }
}

// ========================================================
// ===              Function Definitions                ===
// ========================================================

void updateAngles()
{
  mpu.readData();
}

void initiateTurn()
{
  // Adjust set Point and turning distance based on number of turns made
  sideWallLimit = 100 + floor((turns - 1) / 4) * 300;
  frontWallLimit = 100 + floor(turns / 4) * 300;
  turns++;
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
  double prevAngle = 0;
  double currAngle = mpu.getYaw();
  prevAngle = currAngle;
  double angleTurned = 0;

  turnCorrection.prevTime = millis();
  turnCorrection.prevError = 0;

  while (angleTurned < 90 - rotationalInertia) // Upper bound should be adjusted alongside turnCorrectionPID
  {
    motors.adjust(turnCorrection.calculate(90 - rotationalInertia, angleTurned), 510);
    updateAngles();
    currAngle = mpu.getYaw();
    angleTurned += abs(currAngle - prevAngle);
    prevAngle = currAngle;
  }

  courseCorrection.prevError = 0;
  stopCorrection.prevError = 0;

  courseCorrection.prevTime = millis();
  stopCorrection.prevTime = millis();

  ultrasonicSide.firstReading = true;
  ultrasonicFront.firstReading = true;

  // Stop motors after completing turn
  motors.adjust(255, 0);
}

void checkPitTrap()
{
  if (mpu.getPitch() < -8)
    inPitTrap = true;
  if (mpu.getPitch() > 8 && inPitTrap)
    inPitTrap = false;
}