//I2C LCD uses pins
//5V
//Gnd
//SDA = pin 20
//SCL = pin 21

//HC-SR04 Ultrasonic sensor uses pins
//5V
//Gnd
//Echo = pins 23, 25, 27, 29
//Trig = pins 22, 24, 26, 28

//Add the library
#include <NewPing.h>
#include <LiquidCrystal_I2C.h>

//Define each pin
#define ECHO_PIN1 23 //Front
#define ECHO_PIN2 25 //Left
#define ECHO_PIN3 27 //Back
#define ECHO_PIN4 29 //Right
#define TRIG_PIN1 22 //Front
#define TRIG_PIN2 24 //Left
#define TRIG_PIN3 26 //Back
#define TRIG_PIN4 28 //Right

#define MAX_DISTANCE 80 // Maximum distance (in cm) to ping.

//Initilize the variables
NewPing frontSensor = NewPing(TRIG_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing leftSensor = NewPing(TRIG_PIN2, ECHO_PIN2, MAX_DISTANCE);
NewPing backSensor = NewPing(TRIG_PIN3, ECHO_PIN3, MAX_DISTANCE);
NewPing rightSensor = NewPing(TRIG_PIN4, ECHO_PIN4, MAX_DISTANCE);
LiquidCrystal_I2C lcd(0x27,20,4);

void setup() 
{
  lcd.init(); //Initilize the LCD
  lcd.backlight();  //Turn on the backlight

  //Setup pins
  pinMode(ECHO_PIN1, INPUT);
  pinMode(ECHO_PIN2, INPUT);
  pinMode(ECHO_PIN3, INPUT);
  pinMode(ECHO_PIN4, INPUT);
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(TRIG_PIN3, OUTPUT);
  pinMode(TRIG_PIN4, OUTPUT);
}

void loop() 
{
  int distanceFront = frontSensor.ping_cm(); //Initiates the front sensor and gets the distance.
  int distanceLeft  = leftSensor.ping_cm(); //Initiates the left sensor and gets the distance.
  int distanceBack  = backSensor.ping_cm(); //Initiates the back sensor and gets the distance.
  int distanceRight = rightSensor.ping_cm(); //Initiates the right sensor and gets the distance.

  lcd.clear();  //Clear the LCD
  lcd.setCursor(0,0); //Set cursor to the top row
  lcd.print("F:"); //Print "F: "
  lcd.print(distanceFront); //Print front sensor value
  lcd.print(" L:"); //Print " L: "
  lcd.print(distanceLeft); //Print left sensor value
  lcd.setCursor(0,1); //Set cursor to bottom row
  lcd.print("B:"); //Print "B: "
  lcd.print(distanceBack); //Print back sensor value
  lcd.print(" R:"); //Print " R: "
  lcd.print(distanceRight); //Print right sensor value
  delay(1000); //1 second delay
}
