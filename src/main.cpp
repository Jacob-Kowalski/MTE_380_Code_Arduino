#include <Arduino.h>
#include <Wire.h> 
#include "MPU6050.h"
//defining motor pins
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

//all units in mm
int turns = 1;
bool notDoneCourse = true;
int maxSpeed = 255;

//the wanted distances from each wall
int frontWallLimit = 100;
int sideWallLimit = 50;

//Current measurements
int sideDistance = 0;
int frontDistance = 0;

//The errors for PID
double KP = 1;
double KD = 0;
double KI = 0;
double error = 0;
double previousError = 0;
double PIDCorrection = 0;

//times for PID
double currentTime = 0;
double previousTime = 0;

//for ensuring sampling rate
int frontSensorTime = 0;
int sideSensorTime = 0;

float gyroTime = 0;

float pitch = 0;
float roll = 0;
float yaw = 0;

double PIDController();
void adjustMotors(int maxSpeed, double PIDCorrection);
int getSideDistance();
int getFrontDistance();
void updateAngles();
void initiateTurn();
void turn();
void startMotors();
void stopMotors();

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  //motor Pinout
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

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.begin();
  mpu.calibrateGyro();
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);

  while (getFrontDistance() > 30) {delay(50);}
  startMotors();
}

void loop() {

  updateAngles();
  Serial.print("yaw: ");
  Serial.println(yaw);
  Serial.print("pitch: ");
  Serial.println(pitch);
  Serial.print("roll: ");
  Serial.println(roll);
  Serial.println();
  delay(1000);
  // if (!notDoneCourse) {
  //   stopMotors();
  // } else {
  //   sideDistance = getSideDistance();
  //   frontDistance = getFrontDistance();
  //   updateAngles();

  //   PIDCorrection = PIDController();

  //   if (frontDistance >= frontWallLimit) {
  //     initiateTurn();
  //   } else if (error >= 5) {
  //     adjustMotors(maxSpeed, PIDCorrection);
  //   }
  // }
}

double PIDController() {

  currentTime = micros();

  error = sideWallLimit - sideDistance;
  //1000000 is for seconds can be changes for a nice KD
  double rateError = (error - previousError) * 1000000 / (currentTime - previousTime);

  double overallGain = KP * error + KD * rateError;

  previousError = error;
  previousTime = currentTime;

  return overallGain;
}

void adjustMotors(int maxSpeed, double correction) {
  uint8_t leftSpeed = abs((correction > 0) ? maxSpeed : maxSpeed * (float)(1.0 - abs(correction) / 255.0));
  uint8_t rightSpeed = abs((correction < 0) ? maxSpeed : maxSpeed * (float)(1.0 - abs(correction) / 255.0));

  if (leftSpeed == 0) leftSpeed = 0;
  else if (leftSpeed < 115) leftSpeed = 115;
  if (rightSpeed == 0) rightSpeed = 0;
  else if (rightSpeed < 115) rightSpeed = 115;

  if (maxSpeed > 0) {
    analogWrite(BACK_LEFT_FORWARD, leftSpeed);
    analogWrite(BACK_LEFT_BACKWARD, 0);
    analogWrite(FRONT_LEFT_FORWARD, leftSpeed);
    analogWrite(FRONT_LEFT_BACKWARD, 0);
    analogWrite(BACK_RIGHT_FORWARD, rightSpeed);
    analogWrite(BACK_RIGHT_BACKWARD, 0);
    analogWrite(FRONT_RIGHT_FORWARD, rightSpeed);
    analogWrite(FRONT_RIGHT_BACKWARD, 0);
  } else {
    analogWrite(BACK_LEFT_FORWARD, 0);
    analogWrite(BACK_LEFT_BACKWARD, leftSpeed);
    analogWrite(FRONT_LEFT_FORWARD, 0);
    analogWrite(FRONT_LEFT_BACKWARD, leftSpeed);
    analogWrite(BACK_RIGHT_FORWARD, 0);
    analogWrite(BACK_RIGHT_BACKWARD, rightSpeed);
    analogWrite(FRONT_RIGHT_FORWARD, 0);
    analogWrite(FRONT_RIGHT_BACKWARD, rightSpeed);
  }
}

int getSideDistance() {
  //cheching delay time for max sensor
  if (millis() - sideSensorTime < 25) {
    delay(25 - (millis() - sideSensorTime));
  }

  digitalWrite(pingPinSide, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPinSide, LOW);
  double duration = pulseIn(echoPinSide, HIGH);

  sideSensorTime = millis();
  int time = (duration / 58.0 * 10);
  return time;
}

int getFrontDistance() {
  //cheching delay time for max sensor
  if (millis() - frontSensorTime < 25) {
    delay(25 - (millis() - frontSensorTime));
  }

  digitalWrite(TRIGGER_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_FRONT, LOW);
  double duration = pulseIn(ECHO_FRONT, HIGH);
  // Serial.println(duration);
  // Serial.println("Duration");
  //updating last use time
  frontSensorTime = millis();
  int time = (duration / 58.0 * 10);
  return time;
}

void updateAngles() {
  int timeStep = micros() - gyroTime;
  Vector norm = mpu.readNormalizeGyro();
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  gyroTime = micros();
}

void initiateTurn() {
  // Adjust set Point and turning distance based on number of turns made
  sideWallLimit = 0.03 + floor((turns - 1) / 4) * 0.30;
  frontWallLimit = 0.1 + floor(turns / 4) * 0.30;
  turns++;
  if ((turns - 1) == 11) {
    // shut off motors
    notDoneCourse = 0;
  } else {
    turn();
  }
}

void turn() {
  //motor turns

  while (yaw < 90) {
    updateAngles();
  }

  //stuff here to go straight
  yaw = 0;
}

void startMotors() {
  digitalWrite(BACK_LEFT_FORWARD, HIGH);
  digitalWrite(BACK_LEFT_BACKWARD, LOW);
  digitalWrite(FRONT_LEFT_FORWARD, HIGH);
  digitalWrite(FRONT_LEFT_BACKWARD, LOW);
  digitalWrite(BACK_RIGHT_FORWARD, HIGH);
  digitalWrite(BACK_RIGHT_BACKWARD, LOW);
  digitalWrite(FRONT_RIGHT_FORWARD, HIGH);
  digitalWrite(FRONT_RIGHT_BACKWARD, LOW);
}

void stopMotors() {
  digitalWrite(BACK_LEFT_FORWARD, LOW);
  digitalWrite(BACK_LEFT_BACKWARD, LOW);
  digitalWrite(FRONT_LEFT_FORWARD, LOW);
  digitalWrite(FRONT_LEFT_BACKWARD, LOW);
  digitalWrite(BACK_RIGHT_FORWARD, LOW);
  digitalWrite(BACK_RIGHT_BACKWARD, LOW);
  digitalWrite(FRONT_RIGHT_FORWARD, LOW);
  digitalWrite(FRONT_RIGHT_BACKWARD, LOW);
}
