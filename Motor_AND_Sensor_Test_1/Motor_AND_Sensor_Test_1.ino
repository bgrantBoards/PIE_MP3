/* Motor_AND_Sensor_Test_1.ino
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
int initialSpeed=30; // set both left and right initial speeds to this value
int turn = initialSpeed-2; // turn amount

int sensorPinL = A0;    // select the input pin for the left sensor
int sensorPinR = A2;    // select the input pin for the right
int sensorValueL = 0;  // variable to store the value coming from the left sensor
int sensorValueR = 0;  // variable to store the value coming from the right sensor

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
  sensorValueL= analogRead(sensorPinL);
  Serial.print(sensorValueL);
  Serial.print("\n");
  bool onTapeL = sensorValueL > 750;
  Serial.print(onTapeL);
  Serial.print("\n");
  sensorValueR = analogRead(sensorPinR);
  Serial.print(sensorValueR);
  Serial.print("\n");
  bool onTapeR = sensorValueR > 750;
  Serial.print(onTapeR);
  Serial.print("\n");

  // if left sensor on tape turn left
  if (onTapeL == 1 and onTapeR == 0) {
      myMotorL->setSpeed(initialSpeed-turn);
      myMotorR->setSpeed(initialSpeed);
      myMotorL->run(FORWARD);
      myMotorR->run(FORWARD);
  }

  // if right sensor on tape turn right
  else if (onTapeR == 1 and onTapeL == 0) {
      myMotorL->setSpeed(initialSpeed);
      myMotorR->setSpeed(initialSpeed-turn);
      myMotorL->run(FORWARD);
      myMotorR->run(FORWARD);
  }
  
  // if both sensors on tape stop robot 
  else if (onTapeL == 1 and onTapeR == 1) {
      myMotorL->run(RELEASE);
      myMotorR->run(RELEASE);
  }

  // if not on tape, continue
  else {
      myMotorL->setSpeed(initialSpeed);
      myMotorR->setSpeed(initialSpeed);
      myMotorL->run(FORWARD);
      myMotorR->run(FORWARD);
  }
      
  delay(25); // wait 50 [ms] before checking again
}
