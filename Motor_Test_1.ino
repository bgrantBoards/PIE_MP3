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
int initialSpeed=50; // set both left and right initial speeds to this value

// initialize motor shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // create Adafruit_MotorShield object
Adafruit_DCMotor *myMotorL = AFMS.getMotor(leftMotor); // create DC motor object (left)
Adafruit_DCMotor *myMotorR = AFMS.getMotor(rightMotor); // create DC motor object (right)

void setup() {
  AFMS.begin(); // start motor driver
}

void loop() {
  myMotorL->setSpeed(initialSpeed); // set left motor speed
  myMotorR->setSpeed(initialSpeed); // set right motor speed
  myMotorL->run(FORWARD); // run motor (FORWARD, BACKWARD, or RELEASE)
  myMotorR->run(FORWARD); // run motor (FORWARD, BACKWARD, or RELEASE)
  
  delay(5000);

  // stop motors
  myMotorL->run(RELEASE); // left
  myMotorR->run(RELEASE); // right

  delay(500);

  // turn right at slow speed for 1 [s]
  myMotorL->setSpeed(25);
  myMotorR->setSpeed(25);
  myMotorL->run(FORWARD);
  myMotorR->run(BACKWARD);
  delay(1000);

  // stop motors
  myMotorL->run(RELEASE);
  myMotorR->run(RELEASE);

  // drive backwards at 2x initial speed for 2 seconds
  myMotorL->setSpeed(initialSpeed*2);
  myMotorR->setSpeed(initialSpeed*2);
  myMotorL->run(BACKWARD);
  myMotorR->run(BACKWARD);
  delay(2000);

  // stop motors
  myMotorL->run(RELEASE);
  myMotorR->run(RELEASE);

  delay(5000);
}
