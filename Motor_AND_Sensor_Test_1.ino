/* Motor_Test_1.ino
 *  Basic code to test out the two motors of our line-following robot
 *  Myles Lack-Zell
 *  9/30/21
 */

// load motor driver libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// define variables
int leftMotor=2; // port number for left motor
int rightMotor=1; // port number for right motor
int initialSpeed=25; // set both left and right initial speeds to this value

int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

// initialize motor shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // create Adafruit_MotorShield object
Adafruit_DCMotor *myMotorL = AFMS.getMotor(leftMotor); // create DC motor object (left)
Adafruit_DCMotor *myMotorR = AFMS.getMotor(rightMotor); // create DC motor object (right)

void setup() {
  AFMS.begin(); // start motor driver
  Serial.begin(9600);

  myMotorL->setSpeed(initialSpeed); // set left motor speed
  myMotorR->setSpeed(initialSpeed); // set right motor speed
  myMotorL->run(FORWARD); // run motor (FORWARD, BACKWARD, or RELEASE)
  myMotorR->run(FORWARD); // run motor (FORWARD, BACKWARD, or RELEASE)
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  Serial.print(sensorValue);
  Serial.print("\n");
  bool onTape = sensorValue > 750;
  Serial.print(onTape);
  Serial.print("\n");

  // turn right at slow speed for 1 [s]
  if (onTape == 1) {
      myMotorL->setSpeed(initialSpeed);
      myMotorR->setSpeed(initialSpeed-10);
      myMotorL->run(FORWARD);
      myMotorR->run(FORWARD);
  }
  else {
      myMotorL->setSpeed(initialSpeed);
      myMotorR->setSpeed(initialSpeed);
      myMotorL->run(FORWARD);
      myMotorR->run(FORWARD);
  }
  delay(100);
}
