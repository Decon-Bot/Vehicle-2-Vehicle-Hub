/*
motorValue *******
Reverse: motorValue < 80
Stop: motorValue = 80
Forward: motorValue > 80
steerValue *******
Max Right: steerValue = 40
Right: steerValue < 97
Straight: steerValue = 97
Left: steerValue > 97
Max Left: steerValue = 160
*/

//Motor Servo uses pins
//Gnd
//Data = pin 8

//Steering Servo uses pins
//5V
//Gnd
//Data = pin 7

//Add the library
#include <Servo.h>

//Define each pin
#define MOTOR_PIN 8
#define SERVO_PIN 7

//Initilize the servos
Servo motor;
Servo steer;

//Initilize the abase stop and straight values
int motorValue = 80;
int steerValue = 97;

void setup() 
{
  motor.attach(MOTOR_PIN);  //Attach the motor pin
  motor.write(motorValue); //Set the motor to stop
  steer.attach(SERVO_PIN); //Attach the motor pin
  steer.write(steerValue);  //Set the steering to straight

  delay(2000);  //Delay 2 seconds
  motor.write(90);  //Set the motor to forward
  steer.write(40);  //Set the steering to right
  delay(2000);  //Delay 2 seconds
  motor.write(70);  //Set the motor to reverse
  steer.write(160); //Set the steering to left
  delay(2000);  //Delay 2 seconds
  motor.write(80);  //Set the motor to stop
  steer.write(97);  //Set the steering to straight
  delay(2000);  //Delay 2 seconds
}

void loop() 
{


}
