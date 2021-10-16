/*
Code to control the PIE line follower robot with two IR sensors
*/

// load motor driver libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

class Robot {
  public:
  // adjustable IR sensor threshold value (< is on floor)
  int IR_THRESHOLD = 500;
  int STRAIGHT_SPEED;
  int TURN_VALUE;
  
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

  Robot(int straight_speed, int turn_value){
    // motor go-straight speed
    this->STRAIGHT_SPEED = straight_speed;
    // speed of the slow wheel during a turn
    this->TURN_VALUE = turn_value;
  }

  // IR sensor reading methods
  int read_ir_l(){
    return analogRead(S_L);
  }
  int read_ir_r(){
    return analogRead(S_R);
  }
  bool on_tape_l(){
    return read_ir_l() > IR_THRESHOLD;
  }
  bool on_tape_r(){
    return read_ir_r() > IR_THRESHOLD;
  }

  // motor control methods
  void set_motor_l(int speed){
    if(speed < 0){
      motor_l->setSpeed(-speed);
      motor_l->run(BACKWARD);
    } else {
      motor_l->setSpeed(speed);
      motor_l->run(FORWARD);
    }
  }
  void set_motor_r(int speed){
    if(speed < 0){
      motor_r->setSpeed(-speed);
      motor_r->run(BACKWARD);
    } else {
      motor_r->setSpeed(speed);
      motor_r->run(FORWARD);
    }
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
  void stop(){
    motor_r->run(RELEASE);
    motor_l->run(RELEASE);
  }
  
};

// create Robot object
Robot lineFollower = Robot(50, -30);

void setup() {
  // initialize Serial Output
  lineFollower.AFMS.begin(); // start motor driver
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

  // if both sensors are on line, stop
  if(lineFollower.on_tape_l() && lineFollower.on_tape_r()){
    lineFollower.stop();
  }


  
//  debug
  Serial.print(lineFollower.read_ir_l());
  Serial.print(",");
  Serial.print(lineFollower.read_ir_r());
  Serial.print(",");
  Serial.print(lineFollower.on_tape_l());
  Serial.print(",");
  Serial.println(lineFollower.on_tape_r());

  // read serial inputs for straight speed and turn speed
  if(Serial.available()){
    int straight_speed = Serial.readStringUntil(',').toInt();
    int turn_speed = Serial.readStringUntil('\n').toInt();
 
    lineFollower.STRAIGHT_SPEED = straight_speed;
    lineFollower.TURN_VALUE = turn_speed;

    Serial.print("Straight: ");
    Serial.print(lineFollower.STRAIGHT_SPEED);
    Serial.print(" Turn: ");
    Serial.println(lineFollower.TURN_VALUE);
  }
  
  delay(10); // delay 50 ms
}
