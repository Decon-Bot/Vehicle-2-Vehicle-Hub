//I2C LCD uses pins
//5V
//Gnd
//SDA = pin 20
//SCL = pin 21

//Add the library
#include <LiquidCrystal_I2C.h>

//Initilize I2C variable
LiquidCrystal_I2C lcd(0x27,20,4);

void setup() 
{
  lcd.init(); //Initilize the LCD
  lcd.backlight();  //Turn on the backlight

  lcd.clear();  //Clear the LCD
  lcd.setCursor(0,0); //Set cursor to the top row
  lcd.print("Hello"); //Print "Hello"
  lcd.setCursor(0,1); //Set cursor to bottom row
  lcd.print("World"); //Print "World"
}

void loop() 
{
  
}
