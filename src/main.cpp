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
int frontWallLimit = 150;
int sideWallLimit = 100;

// Current ultrasonic measurements
int sideDistance = 0;
int frontDistance = 0;

bool inPitTrap = false;

// Initialize Sensor objects
Gyro mpu;
Ultrasonic ultrasonicFront('f');
Ultrasonic ultrasonicSide('s');

// Initialize Motor object
Motors motors;

// PID Controllers
PID courseCorrection(-140, 140, 2, 100, 0);
PID stopCorrection(MOTOR_MIN_SPEED, MOTOR_MAX_SPEED, 2.5, 0.5, 0);
PID turnCorrection(MOTOR_MIN_SPEED, MOTOR_MAX_SPEED, 5, 100, 0);

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

    if (frontDistance <= 200 && !inPitTrap) // frontWallLimit)
    {
      initiateTurn();
    }
    else // if (error >= 5)
    {
      motors.adjust(stopCorrection.calculate(frontDistance, frontWallLimit), courseCorrection.calculate(sideWallLimit, sideDistance));
    }
  }
}

// ========================================================
// ===               Function Prototypes                ===
// ========================================================

void updateAngles();
void initiateTurn();
void turn();
void checkPitTrap();

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
  sideWallLimit = 30 + floor((turns - 1) / 4) * 300;
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
  double angleTurned = 0;

  while (angleTurned < 90) // Upper bound should be adjusted alongside turnCorrectionPID
  {
    motors.adjust(turnCorrection.calculate(90, angleTurned), 510);
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