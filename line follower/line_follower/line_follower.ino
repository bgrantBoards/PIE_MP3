/*
Code to control the PIE line follower robot with two IR sensors
*/

// load motor driver libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

class Robot {
  // adjustable IR sensor threshold value (< is on floor)
  const int IR_THRESHOLD = 750;
  // motor go-straight speed
  const int STRAIGHT_SPEED = 30;
  // speed of the slow wheel during a turn
  const int TURN_VALUE = 2;
  
  // declare sensor pins
  const int S_L = A0;
  const int S_R = A2;
  // declare motor pins
  const int M_L_PIN = 2;
  const int M_R_PIN = 1;
  
  // initialize motor shield
  Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // create Adafruit_MotorShield object
  // create motor objects
  Adafruit_DCMotor *motor_l = AFMS.getMotor(M_L_PIN);
  Adafruit_DCMotor *motor_r = AFMS.getMotor(M_R_PIN);

  public:

  // IR sensor reading methods
  int read_ir_l(){
    return analogRead(S_L);
  }
  int read_ir_r(){
    return analogRead(S_L);
  }
  bool on_tape_l(){
    return read_ir_l() > IR_THRESHOLD;
  }
  bool on_tape_r(){
    return read_ir_r() > IR_THRESHOLD;
  }

  // motor control methods
  void set_motor_l(int speed){
    motor_l->setSpeed(speed);
  }
  void set_motor_r(int speed){
    motor_r->setSpeed(speed);
  }

  // movement methods
  void go_straight(){
    set_motor_l(STRAIGHT_SPEED);
    set_motor_r(STRAIGHT_SPEED);
  }
  void turn_l(){
    set_motor_l(TURN_VALUE);
    set_motor_r(STRAIGHT_SPEED);
  }
  void turn_r(){
    set_motor_l(STRAIGHT_SPEED);
    set_motor_r(TURN_VALUE);
  }
  
};

// create Robot object
Robot lineFollower;

void setup() {
  // initialize Serial Output
  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
  // if left sensor is on line, turn left
  if(lineFollower.on_tape_l()){
    lineFollower.turn_l();
  }

  // if right sensor is on line, turn right
  if(lineFollower.on_tape_r()){
    lineFollower.turn_r();
  }

  // if no sensors are on line, go straight
  if(!lineFollower.on_tape_l() && !lineFollower.on_tape_r()){
    lineFollower.go_straight();
  }

  Serial.print("Left IR: " + String(lineFollower.read_ir_l()) + " Right IR: " + String(lineFollower.read_ir_r()));
}
