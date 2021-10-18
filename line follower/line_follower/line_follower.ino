/*
Code to control the PIE line follower robot with two IR sensors
*/

// load motor driver libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

class Robot {
  public:

  // Robot behavior parameters
  int IR_THRESHOLD = 500;  // IR sensor threshold value (< is on floor)
  int STRAIGHT_SPEED = 30; // speed of both wheels while robot is travelling straight
  int TURN_VALUE = -30;    // speed of the slow wheel during a turn

  // Arduino pins
  const int S_L = A0;      // Left IR sensor pin
  const int S_R = A2;      // Right IR sensor pin
  const int M_L_PIN = 2;   // Left DC motor pin
  const int M_R_PIN = 1;   // Right DC motor pin
  
  // Adafruit Motor shield object
  Adafruit_MotorShield AFMS = Adafruit_MotorShield();
  
  // DC motor objects
  Adafruit_DCMotor *motor_l = AFMS.getMotor(M_L_PIN);
  Adafruit_DCMotor *motor_r = AFMS.getMotor(M_R_PIN);

  // motor speed variables (for logging only)
  int speed_m_l = 0;
  int speed_m_r = 0;

  public:
  
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
      // set negative speed
      motor_l->setSpeed(-speed);
      motor_l->run(BACKWARD);
      
      speed_m_l = speed; // for speed logging
    } else {
      // set positive speed
      motor_l->setSpeed(speed);
      motor_l->run(FORWARD);
      
      speed_m_l = speed; // for speed logging
    }
    
    // for 0 speed
    if(!speed){
      motor_l->run(RELEASE);
    }
  }
  void set_motor_r(int speed){
    if(speed < 0){
      // set negative speed
      motor_r->setSpeed(-speed);
      motor_r->run(BACKWARD);
      
      speed_m_r = speed; // for speed logging
    } else {
      // set positive speed
      motor_r->setSpeed(speed);
      motor_r->run(FORWARD);
      
      speed_m_r = speed; // for speed logging
    }

    // for 0 speed
    if(!speed){
      motor_l->run(RELEASE);
    }
  }

  // Robot behavior methods
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
  void stop_motors(){
    set_motor_l(0);
    set_motor_r(0);
  }

  // serial input method
  void handle_input(){
    // serial input format: <straightspeed>,<turnspeed>,<threshold>
    if(Serial.available()){
      int straight_speed = Serial.readStringUntil(',').toInt();
      int turn_speed = Serial.readStringUntil(',').toInt();
      int treshold = Serial.readStringUntil('\n').toInt();
   
      STRAIGHT_SPEED = straight_speed;
      TURN_VALUE = turn_speed;
      IR_THRESHOLD = treshold;
    }
  }

  // data logging method
  void log_state(){
    Serial.print(read_ir_l());
    Serial.print(",");
    Serial.print(read_ir_r());
    Serial.print(",");
    
    Serial.print(on_tape_l());
    Serial.print(",");
    Serial.print(on_tape_r());
    Serial.print(",");
    
    Serial.print(speed_m_l);
    Serial.print(",");
    Serial.println(speed_m_r);
  }

  // method for switching between behavior states based on sensor data
  void handle_behavior(){
    // if only left sensor is on line, turn left
    if(on_tape_l() && !on_tape_r()){
      turn_l();
    }
    // if only right sensor is on line, turn right
    if(on_tape_r() && !on_tape_l()){
      turn_r();
    }
    // if no sensors are on line, go straight
    if(!on_tape_l() && !on_tape_r()){
      go_straight();
    }
    // if both sensors are on line, stop
    if(on_tape_l() && on_tape_r()){
      stop_motors();
    }
  }
};

// create Robot object
Robot lineFollower = Robot();

void setup() {
  // initialize Serial Output
  lineFollower.AFMS.begin(); // start motor driver
  Serial.begin(9600);
}

void loop() {
  // switch behavior states according to sensor input
  lineFollower.handle_behavior();

  // read serial inputs for straight speed and turn speed [format: <straightspeed>,<turnspeed>,<threshold>]
  lineFollower.handle_input();

  // log sensor and motor speed data to serial
  lineFollower.log_state();

  delay(5); // delay in ms
}
