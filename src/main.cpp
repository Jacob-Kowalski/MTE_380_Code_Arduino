#include <Arduino.h>
#include <Wire.h>

#include "sensors/gyro/gyro.h"

// =====================================================
// ===               Pin Definitions                 ===
// =====================================================

// defining motor pins
#define FRONT_LEFT_FORWARD 3
#define FRONT_LEFT_BACKWARD 2
#define BACK_LEFT_FORWARD 4
#define BACK_LEFT_BACKWARD 5
#define BACK_RIGHT_FORWARD 7
#define BACK_RIGHT_BACKWARD 6
#define FRONT_RIGHT_FORWARD 8
#define FRONT_RIGHT_BACKWARD 9
#define UTRASONIC_POWER 50

#define TRIGGER_FRONT 12
#define ECHO_FRONT 13

// Trigger Pin of Ultrasonic Sensor
const int pingPinSide = 10;
// const int pingPinFront = 12;

// Echo Pin of Ultrasonic Sensor
const int echoPinSide = 11;
// const int echoPinFront = 13;

// =====================================================
// ===               Global Variables                ===
// =====================================================

// all units in mm

int turns = 1;
bool doneCourse = false;
int maxSpeed = 240;

// the wanted distances from each wall in mm
int frontWallLimit = 250;
int sideWallLimit = 100;

// Current measurements
int sideDistance = 0;
int frontDistance = 0;
int previousSideDistance = 50;
bool firstReading = true;

// The errors for PID
double KP = 800.0 / 1000.0;
double KD = 300.0 / 1000.0;
double KI = 0 / 1000;

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
bool pitTrap = 0;

int angleCount = 0;

// Sensors
gyro MPU;

// ========================================================
// ===               Function Prototypes                ===
// ========================================================

double PIDController();
void adjustMotors(int maxSpeed, double PIDCorrection);
int getSideDistance();
int getFrontDistance();
void updateAngles();
void initiateTurn();
void turn();
void startMotors();
void stopMotors();
void checkPitTrap();

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  }
  // motor Pinout
  pinMode(BACK_LEFT_FORWARD, OUTPUT);
  pinMode(BACK_LEFT_BACKWARD, OUTPUT);
  pinMode(FRONT_LEFT_FORWARD, OUTPUT);
  pinMode(FRONT_LEFT_BACKWARD, OUTPUT);
  pinMode(BACK_RIGHT_FORWARD, OUTPUT);
  pinMode(BACK_RIGHT_BACKWARD, OUTPUT);
  pinMode(FRONT_RIGHT_FORWARD, OUTPUT);
  pinMode(FRONT_RIGHT_BACKWARD, OUTPUT);
  // utrasonic power

  pinMode(UTRASONIC_POWER, OUTPUT);

  delay(100);

  pinMode(TRIGGER_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(pingPinSide, OUTPUT);
  pinMode(echoPinSide, INPUT);
  digitalWrite(TRIGGER_FRONT, LOW);
  digitalWrite(pingPinSide, LOW);
  // ultrasonic power
  digitalWrite(UTRASONIC_POWER, HIGH);

  bool gyroStatus = MPU.init();
  if (gyroStatus)
  {
    Serial.print("MPU initialized successfully");
  }

  delay(2000);

  adjustMotors(maxSpeed, 0);
}

void loop()
{
  if (doneCourse)
  {
    stopMotors();
  }
  else
  {
    sideDistance = getSideDistance();
    frontDistance = getFrontDistance();
    updateAngles();

    PIDCorrection = PIDController();
    checkPitTrap();

    if (frontDistance <= 200 && !pitTrap) // frontWallLimit)
    {
      initiateTurn();
    }
    else // if (error >= 5)
    {
      adjustMotors(maxSpeed, PIDCorrection);
    }
  }
}

double PIDController()
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

void adjustMotors(int maxSpeed, double correction)
{
  int16_t leftSpeed = ((correction > 0) ? maxSpeed : maxSpeed * (float)(1.0 - abs(correction) / 255.0));
  int16_t rightSpeed = ((correction < 0) ? maxSpeed : maxSpeed * (float)(1.0 - abs(correction) / 255.0));

  if (leftSpeed == 0)
    leftSpeed = 0;
  else if (abs(leftSpeed) < 115)
    leftSpeed = (leftSpeed > 0) ? 115 : -115;
  if (rightSpeed == 0)
    rightSpeed = 0;
  else if (abs(rightSpeed) < 115)
    rightSpeed = (rightSpeed > 0) ? 115 : -115;

  if (leftSpeed > 0)
  {
    analogWrite(BACK_LEFT_FORWARD, abs(leftSpeed));
    analogWrite(BACK_LEFT_BACKWARD, 0);
    analogWrite(FRONT_LEFT_FORWARD, abs(leftSpeed));
    analogWrite(FRONT_LEFT_BACKWARD, 0);
  }
  else
  {
    analogWrite(BACK_LEFT_FORWARD, 0);
    analogWrite(BACK_LEFT_BACKWARD, abs(leftSpeed));
    analogWrite(FRONT_LEFT_FORWARD, 0);
    analogWrite(FRONT_LEFT_BACKWARD, abs(leftSpeed));
  }

  if (rightSpeed > 0)
  {
    analogWrite(BACK_RIGHT_FORWARD, abs(rightSpeed));
    analogWrite(BACK_RIGHT_BACKWARD, 0);
    analogWrite(FRONT_RIGHT_FORWARD, abs(rightSpeed));
    analogWrite(FRONT_RIGHT_BACKWARD, 0);
  }
  else
  {
    analogWrite(BACK_RIGHT_FORWARD, 0);
    analogWrite(BACK_RIGHT_BACKWARD, abs(rightSpeed));
    analogWrite(FRONT_RIGHT_FORWARD, 0);
    analogWrite(FRONT_RIGHT_BACKWARD, abs(rightSpeed));
  }
}

int getSideDistance()
{
  // cheching delay time for max sensor
  if (millis() - sideSensorTime <= 25)
  {
    delay(25 - (millis() - sideSensorTime));
  }
  previousSideDistance = sideDistance;
  digitalWrite(pingPinSide, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPinSide, LOW);
  double duration = pulseIn(echoPinSide, HIGH);

  sideSensorTime = millis();
  int distance = (duration / 58.0 * 10);

  if (abs(distance - sideDistance) > 600 && !firstReading)
  {
    distance = sideDistance;
  }
  firstReading = false;
  return distance;
}

int getFrontDistance()
{
  // cheching delay time for max sensor
  if (millis() - frontSensorTime < 25)
  {
    delay(25 - (millis() - frontSensorTime));
  }

  digitalWrite(TRIGGER_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_FRONT, LOW);
  double duration = pulseIn(ECHO_FRONT, HIGH);
  // Serial.println(duration);
  // Serial.println("Duration");
  // updating last use time
  frontSensorTime = millis();
  int time = (duration / 58.0 * 10);
  return time;
}

void updateAngles()
{
  MPU.readData();
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
    Serial.print("Course completed");
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
  // motor turns
  adjustMotors(150, 255 + 150);

  // float angle = 90 * (((turns) % 4) + 1) - 185;
  float initialAngle = MPU.getYaw();

  // while (ypr[0] * 180 / M_PI < angle || angle - ypr[0] * 180 / M_PI < -40)
  while (abs(abs(initialAngle) - abs(MPU.getYaw())) < 80)
  {
    updateAngles();
  }

  adjustMotors(maxSpeed, 0);
}

void startMotors()
{
  digitalWrite(BACK_LEFT_FORWARD, HIGH);
  digitalWrite(BACK_LEFT_BACKWARD, LOW);
  digitalWrite(FRONT_LEFT_FORWARD, HIGH);
  digitalWrite(FRONT_LEFT_BACKWARD, LOW);
  digitalWrite(BACK_RIGHT_FORWARD, HIGH);
  digitalWrite(BACK_RIGHT_BACKWARD, LOW);
  digitalWrite(FRONT_RIGHT_FORWARD, HIGH);
  digitalWrite(FRONT_RIGHT_BACKWARD, LOW);
}

void stopMotors()
{
  digitalWrite(BACK_LEFT_FORWARD, LOW);
  digitalWrite(BACK_LEFT_BACKWARD, LOW);
  digitalWrite(FRONT_LEFT_FORWARD, LOW);
  digitalWrite(FRONT_LEFT_BACKWARD, LOW);
  digitalWrite(BACK_RIGHT_FORWARD, LOW);
  digitalWrite(BACK_RIGHT_BACKWARD, LOW);
  digitalWrite(FRONT_RIGHT_FORWARD, LOW);
  digitalWrite(FRONT_RIGHT_BACKWARD, LOW);
}

void checkPitTrap()
{
  if (MPU.getPitch() < -10)
    pitTrap = true;
  if (MPU.getPitch() > 10 && pitTrap)
    pitTrap = false;
}