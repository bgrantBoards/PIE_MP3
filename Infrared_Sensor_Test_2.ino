/*
  Analog Input
  Demonstrates analog input by reading an analog sensor on analog pin 0 and
  turning on and off a light emitting diode(LED) connected to digital pin 13.
  The amount of time the LED will be on and off depends on the value obtained
  by analogRead().
  The circuit:
  - potentiometer
    center pin of the potentiometer to the analog input 0
    one side pin (either one) to ground
    the other side pin to +5V
  - LED
    anode (long leg) attached to digital output 13 through 220 ohm resistor
    cathode (short leg) attached to ground
  - Note: because most Arduinos have a built-in LED attached to pin 13 on the
    board, the LED is optional.
  created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe
  This example code is in the public domain.
  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogInput
*/

int sensorPinL = A0;    // select the input pin for the left sensor
int sensorPinR = A2;    // select the input pin for the right sensor
int sensorValueL = 0;  // variable to store the value coming from the left sensor
int sensorValueR = 0;  // variable to store the value coming from the right sensor

void setup() {
  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  sensorValueL = analogRead(sensorPinL);
  sensorValueR = analogRead(sensorPinR);
  Serial.print(sensorValueL);
  Serial.print("\n");
  Serial.print(sensorValueR);
  Serial.print("\n");
  Serial.print("\n");
  delay(100);
}
