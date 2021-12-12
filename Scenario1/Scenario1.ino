#include <Arduino.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

void control_new();
int pullWantedBit(uint64_t , int );
void SendDataConversion();
void ReceivedDataConversion();
void sensorSweep();
void evalUltra();
void bitmap();
void receiveData();
void sendData();
void dataEx();
void laneChange_old();
void drive_old();
void driveCenterRight();
void driveCenterLeft();
void getUltra();
void ultrasonicRight();
void ultrasonicBack();
void ultrasonicLeft();
void ultrasonicFront();
void csInt(String , String , int , int , int , int , int , int );
void cs(String , String , int , int , int );
void csCarInfo(String , int , String , int , String , int , String , int , String , int , String );
void evalColor();
void colorCheck();
void getColor_old();
int getBluePW();
int getGreenPW();
void drive();
void control();
void ultrasonictest();
void modeRx();
size_t print64(Print* , uint64_t);
size_t print64(Print* pr, uint64_t n);
int pullWantedBits(uint64_t , int , int );
void SendDataConversion();
void ReceivedDataConversion();
void OtherCarLocation();
void getUltrasonic();
void getColor();
void button();
void lineFollow();
void laneChange();
void csCar(String , String , String , String , String , String , int );
void bitmap8(uint8_t);
void lineFollow_noLCD();
void red();
void green();
void blue();
void LED_off();
void steerTest();
void speedTest();
void commsCheck();

//Radio Pins
#define CE_PIN 51      // RF CE Pin
#define CSN_PIN 50     // RF CSN Pin

//LEDs and Buttons
#define LED_PIN1 47    // LED Red
#define BUTTON_PIN1 46 // Button Red

#define LED_PIN2 43    // LED Green
#define BUTTON_PIN2 42 // Button Green

#define LED_PIN4 39    // Tri LED Red
#define LED_PIN5 38    // Tri LED Green
#define LED_PIN6 37    // Tri LED Blue

#define LED_PIN3 35    // LED Blue
#define BUTTON_PIN3 34 // Button Blue

//define ultrasonic sensor pins
#define ECHO_PIN4 29   // Right
#define TRIG_PIN4 28   // Right
#define ECHO_PIN3 27   // Back
#define TRIG_PIN3 26   // Back
#define ECHO_PIN2 25   // Left
#define TRIG_PIN2 24   // Left
#define ECHO_PIN1 23   // Front
#define TRIG_PIN1 22   // Front  

//Color sensor LED
#define CS_LED 9

//define motor and servo pins
#define MOTOR_PIN 8
#define SERVO_PIN 7

//define color sensor pins
#define COLOR_S3_PIN 6
#define COLOR_S2_PIN 5
#define COLOR_S1_PIN 4
#define COLOR_S0_PIN 3
#define COLOR_OUT_PIN 2

#define MAX_DISTANCE 200  // Maximum distance (in cm) to ping.

// Calibration Values
int redMin = 255;         // Red minimum value
int redMax = 0;           // Red maximum value
int greenMin = 255;       // Green minimum value
int greenMax = 0;         // Green maximum value
int blueMin = 255;        // Blue minimum value
int blueMax = 0;          // Blue maximum value
// Calibration Values

// Calibration set, max/min Values
int A=2000;               // manual wait time
int B=300;                // Scan color delay
int colorAccuracy=15;     // Iterations of loop
int colorScanDelay=800;   // Drive time delay to move to next color
int colorSpread=50;

int rtrmin=255;
int rtrmax=0;
int gtrmin=255;
int gtrmax=0;
int btrmin=255;
int btrmax=0;

int rtgmin=255;
int rtgmax=0;
int gtgmin=255;
int gtgmax=0;
int btgmin=255;
int btgmax=0;

int rtbmin=255;
int rtbmax=0;
int gtbmin=255;
int gtbmax=0;
int btbmin=255;
int btbmax=0;

int rtfmin=255;
int rtfmax=0;
int gtfmin=255;
int gtfmax=0;
int btfmin=255;
int btfmax=0;

int colorCheckRed = 0;
int colorCheckGreen = 0;
int colorCheckBlue =0;
// Calibration set, max/min Values

// Variables for Color Pulse Width Measurements
int PW;
int redPW = 0;
int greenPW = 0;
int bluePW = 0;
// Variables for Color Pulse Width Measurements

// Variables for Color sensor values
int redValue;
int greenValue;
int blueValue;
// Variables for Color sensor values

int colorDetected = 0;     // 0-White, 1-Black, 2-Red, 3-Green, 4-Blue, 5-Unknow

int motorValue = 80;       // Stop
int steerValue = 86;       // Center

int button1State=HIGH;     // Red
int button2State=HIGH;     // Green
int button3State=HIGH;     // Blue

String color[] = {"White", "Black", "Red", "Green","Blue", "Unknown", "R", "G", "B", "", "Floor"};
String info[] = {"--", "Ready", "Measuring", "Set", "Complete", "Track", "Skip", "Press", "# =", "+1",  "Color Check", "", "Dis:", "Car:", "Ln:", "Dir:", "TX", "RX", "FU:", "LU:", "BU:", "RU:", "Scenario:", "Run"};

char firstRow1;
char firstRow2;
char firstRow3;
char firstRow4;
char firstRow5;
char secondRow1;
char secondRow2;
char secondRow3;
int secondRowInt1;
int secondRowInt2;
int secondRowInt3;
int secondRowInt4;
int secondRowInt5;
int secondRowInt6;

float durationFront, distanceFront, durationLeft, distanceLeft,durationBack, distanceBack,durationRight, distanceRight;
int objectFrontDis = 0;
int objectLeftDis = 0;
int objectBackDis = 0;
int objectRightDis = 0;

int straight = 86;
int correctLeft = 120;
int correctRight = 50;
int correctTime = 400;
int laneChangeLeft= 102;
int laneChangeRight= 70;
int reverseLeft= 50;
int reverseRight= 115;

int speedStop=80;
int speedFull=140;
int speedOneQuarter=100;
int speedHalf=110;
int speedThreeQuarter=115;

int carSpeedStop[2]= {80,80};
int carSpeedSlow[2] = {92, 89};
int carSpeed[2] = {94, 91};
int carSpeedFast[2] = {98, 94};
int carServoStraight[2] = {88, 96};
int carServoLeft[2] = {105, 120};
int carServoRight[2] = {69, 72};
int carServoLeftMax[2] = {114, 150};
int carServoRightMax[2] = {55, 42};
int trial = 80;

int goDrive=0;
int direct = 0;

int i=0;
int servoCheckLoop=1;

int distanceMarker = 1;     // green
int leftLaneMarker = 2;     // red
int rightLaneMarker = 3;    // blue

int disColor = 0;

Servo motor;
Servo steer;

LiquidCrystal_I2C lcd(0x27,16,2);

RF24 radio (CE_PIN, CSN_PIN);                 // 51 CE, 50 CSN

//Transmitted and Received Data
uint8_t priority = 0;
uint8_t radioNumber_Lane_Ultrasonic[3] = {0,0,0};
uint8_t lane[3] = {0,0,0};                     // left lane 0, right lane 1
uint8_t distance[3] = {0,0,0}; 
uint8_t direction[3] = {0,0,0}; 
uint8_t scenario[3] = {0,0,0}; 
uint8_t frontUltrasonicDistance[3] = {100,100,100};   // Distance Front Sensor
uint8_t leftUltrasonicDistance[3]  = {0,0,0};   // Distance Left Sensor
uint8_t backUltrasonicDistance[3]  = {0,0,0};   // Distance Back Sensor
uint8_t rightUltrasonicDistance[3] = {0,0,0};   // Distance Rigth Sensor
bool ultHitFront[3] = {0,0,0};                  // Object in front
bool ultHitLeft[3]  = {0,0,0};                  // Object on the left
bool ultHitBack[3]  = {0,0,0};                 // Object in back
bool ultHitRight[3] = {0,0,0};                  // Object on the right
char otherCarDirection = 0;                   // 0 = north, 1 = north west, 2 = west, 3 = south west, 4 = south, 5 = south east, 6 = east, 7 = north east

NewPing frontSensor = NewPing(TRIG_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing leftSensor = NewPing(TRIG_PIN2, ECHO_PIN2, MAX_DISTANCE);
NewPing backSensor = NewPing(TRIG_PIN3, ECHO_PIN3, MAX_DISTANCE);
NewPing rightSensor = NewPing(TRIG_PIN4, ECHO_PIN4, MAX_DISTANCE);

uint64_t receivedInformation = 0;             // used to store payload from receive module
uint64_t payload[3] = {0,0,0};                         // used to store payload from receive module
uint8_t address[][6] = {"1Node","2Node","3Node"};
int otherCar = 0;

int carSet = 0;
int radioNumber = 0;                  // 0 uses address[0] to transmit, 1 uses address[1] to transmit
int tempRadioNumber = 0;
bool role = false;
int roleText[2] = {17,17};
int usNumber[2] = {18,18};//18 front, 19 Left, 20 Back, 21 Right
int vals = -1;

uint8_t Txaddress;
uint64_t tpayload;

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long interval[2] = {6000,5100};

int laneChangeCount = 0;
int distanceLock = 0;
int laneLock = 0;

//car 0 white, car 1 black
//Full charge
//white:
// Speed Forward: 105
// Speed Back: 68
// Center: 88
// Left max: 114
// Right max: 55
//black:
// Speed Forward: 95
// Speed Back: 70
// Center: 96
// Left max: 150
// Right max: 42

//int carspeed[] = {0,0}

int cycleCount = 0;
unsigned long timer = 0;
unsigned long timerSet = 0;

void setup() 
{
  Serial.begin(115200);
  while (!Serial) 
  {
    // some boards need to wait to ensure access to serial over USB
  }

  //Configure LCD
  lcd.init();
  lcd.backlight();

  //Configure servos
  motor.attach(MOTOR_PIN);
  motor.write(carSpeedStop[radioNumber]);
  steer.attach(SERVO_PIN);
  steer.write(steerValue);

  // Set S0 - S3 as outputs // Set Frequency scaling to S0-High S1-Low 20%
  pinMode(COLOR_S0_PIN, OUTPUT);   //pin modes
  pinMode(COLOR_S1_PIN, OUTPUT);
  pinMode(COLOR_S2_PIN, OUTPUT);
  pinMode(COLOR_S3_PIN, OUTPUT);
  pinMode(COLOR_OUT_PIN, INPUT);
  digitalWrite(COLOR_S0_PIN, HIGH); //Putting S0/S1 on HIGH/HIGH levels means the output frequency scalling is at 100% (recommended)
  digitalWrite(COLOR_S1_PIN, LOW); //LOW/LOW is off HIGH/LOW is 20% and LOW/HIGH is  2%
  
  //Configure ultrasonic 
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(TRIG_PIN3, OUTPUT);
  pinMode(TRIG_PIN4, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(ECHO_PIN2, INPUT);
  pinMode(ECHO_PIN3, INPUT);
  pinMode(ECHO_PIN4, INPUT);

  //Configure Button
  pinMode(BUTTON_PIN1, INPUT_PULLUP);
  pinMode(BUTTON_PIN2, INPUT_PULLUP);

  //Configure LED
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);
  pinMode(LED_PIN4, OUTPUT);
  pinMode(LED_PIN5, OUTPUT);

  digitalWrite(LED_PIN3, LOW);
  digitalWrite(LED_PIN4, LOW);
  digitalWrite(LED_PIN5, LOW);
  
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) 
  {
    Serial.println(F("radio hardware is not responding!!"));
    while(1){} // hold in infinite loop
  }
  //radio number
  csCar(info[7],color[6],info[9],color[7],info[3],info[13],radioNumber);
  while(carSet == 0)
  {
    button();
      if (button1State==LOW) // Red button press: +1
      {                
        radioNumber++;
        csCar(info[7],color[6],info[9],color[7],info[3],info[13],radioNumber);
      } 
      else if(button2State==LOW)// Green button press: Set
      {           
        carSet=1;
      }
    delay(120);
  }

  //lane number
  csCar(info[7],color[6],info[9],color[7],info[3],info[14],lane[radioNumber]);
  while(carSet == 1)
  {
    button();
      if (button1State==LOW)
      {                 
        lane[radioNumber]++;
        csCar(info[7],color[6],info[9],color[7],info[3],info[14],lane[radioNumber]);
      } 
      else if(button2State==LOW)
      {           
        carSet=2;
      }
    delay(120);
  }

  //direction number
  csCar(info[7],color[6],info[9],color[7],info[3],info[15],direction[radioNumber]);
  while(carSet == 2)
  {
    button();
      if (button1State==LOW)
      {                 
        direction[radioNumber]++;
        csCar(info[7],color[6],info[9],color[7],info[3],info[15],direction[radioNumber]);
      } 
      else if(button2State==LOW)
      {           
        carSet=3;
      }
    delay(120);
  }
  
  //senario
  csCar(info[7],color[6],info[9],color[7],info[3],info[22],scenario[radioNumber]);
  while(carSet == 3)
  {
    button();
      if (button1State==LOW)
      {                 
        scenario[radioNumber]++;
        csCar(info[7],color[6],info[9],color[7],info[3],info[22],scenario[radioNumber]);
      } 
      else if(button2State==LOW)
      {           
        carSet=4;
      }
    delay(120);
  }

  //color check
  csCar(info[7],color[6],info[23],color[7],info[6],info[10],0);
  while(carSet == 4)
  {
    button();
      if (button1State==LOW)
      {                 
        colorCheck();
        csCar(info[10],info[4],info[11],info[11],info[11],info[11],0);
      } 
      else if(button2State==LOW)
      {           //Green button press: Set
        carSet=5;
      }
    delay(120);  
  }
  csCarInfo(info[13], radioNumber, info[14], lane[radioNumber], info[15], direction[radioNumber], info[12], distance[radioNumber], info[usNumber[radioNumber]], frontUltrasonicDistance[radioNumber],info[roleText[radioNumber]]);
  
  radio.setPALevel(RF24_PA_MIN);       //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
  //radio.setCRCLength(RF24_CRC_16);
  radio.setChannel(35);
  radio.setDataRate(RF24_250KBPS); //
  radio.setRetries(3,5); // delay, count
  radio.setPayloadSize(sizeof(payload)); // float datatype occupies xx bytes

    
  radio.openWritingPipe(address[radioNumber]);
  if(radioNumber==0)
  {
    radio.openReadingPipe(1, address[1]);
  }
  else if(radioNumber==1)
  {
    radio.openReadingPipe(1, address[0]);
  }
  
  printf_begin();             // needed only once for printing details
  radio.printDetails();       // (smaller) function that prints raw register values
  radio.printPrettyDetails(); // (larger) function that prints human readable data
 
  motor.write(carSpeedStop[radioNumber]);
  steer.write(carServoStraight[radioNumber]);
  
  if (radioNumber == 0)
  {
    priority = 1 ;
  }
}

void loop()
{
  
  button();
  //speedTest();
  //steerTest();
  //remoteSerial();
  evalColor();
  lineFollow_noLCD();
  ultrasonictest();
  modeRx();
  dataEx();

  if (button1State==LOW)    //Red button press:
  {
    //commsCheck();
    motor.write(carSpeedSlow[radioNumber]);  
  }
  else if(button2State==LOW)//Green button press:
  {
    distance[radioNumber] = 0;
    motor.write(carSpeedStop[radioNumber]);
  } 
  else if(button3State==LOW)//Blue button press:
  {
      
  }
  delay(20);
  
}

void laneChange()
{
  if (lane[radioNumber] == 0)
  {
    if (timer < timerSet + 3100)
    {
      steerValue =  40;
    }
    else if (timer < timerSet + 4000)
    {
      steerValue =  60;
    }
    else if (timer < timerSet + 7000)
    {
      steerValue =  150;
    }
  }
  else
  {
    steerValue = 120;
  }
  laneChangeCount++;
  
}

void lineFollow()
{
  lcd.setCursor(0, 1); //Set cursor to bottom row
  if (lane[radioNumber] == 0)
  {
    if (colorDetected == 1 && distanceLock == 0) //green
    {
      distance[radioNumber]++;
      distanceLock = 1;
      lcd.print("Distance inc");
    }
    else if (colorDetected == 2) //Yellow
    {
      steerValue = carServoRight[radioNumber];
      distanceLock = 0;
      lcd.print("Turn right");
    }
    else if(colorDetected == 4) //other color
    {
      steerValue = carServoLeft[radioNumber];
      distanceLock = 0;
      lcd.print("Turn left");
    }
    else
    {
      steerValue = 85;
      lcd.print("Straight");
    }
  }
  else
  {
    if (colorDetected == 1 && distanceLock == 0) //green
    {
      distance[radioNumber]++;
      distanceLock = 1;
      lcd.print("Distance inc");
    }
    else if (colorDetected == 3) //Blue
    {
      steerValue = carServoLeft[radioNumber];
      distanceLock = 0;
      lcd.print("Turn left");
    }
    else if(colorDetected == 4) //other color
    {
      steerValue = carServoRight[radioNumber];
      distanceLock = 0;
      lcd.print("Turn right");
    }
    else
    {
      steerValue = 85;
      lcd.print("Straight");
    }
  }
}

void button()
{
    button1State = digitalRead(BUTTON_PIN1);
    if (button1State == HIGH)
    {
      digitalWrite(LED_PIN1, HIGH);
    } 
    else
    {
      digitalWrite(LED_PIN1, LOW);
    }
    button2State = digitalRead(BUTTON_PIN2);
    if (button2State == HIGH)
    {
      digitalWrite(LED_PIN2, HIGH);
    } 
    else
    {
      digitalWrite(LED_PIN2, LOW);
    }
    delay(20);
}

//Gets the color number that is detected
void getColor()
{
  int Red = 0, Blue = 0, Green = 0; //RGB values
  digitalWrite(COLOR_S2_PIN, LOW);
  digitalWrite(COLOR_S3_PIN, LOW);
  Red = pulseIn(COLOR_OUT_PIN, digitalRead(COLOR_OUT_PIN) == HIGH ? LOW : HIGH);
  delay(5);
  
  digitalWrite(COLOR_S3_PIN, HIGH);
  Blue = pulseIn(COLOR_OUT_PIN, digitalRead(COLOR_OUT_PIN) == HIGH ? LOW : HIGH);
  delay(5);
  
  digitalWrite(COLOR_S2_PIN, HIGH);
  Green = pulseIn(COLOR_OUT_PIN, digitalRead(COLOR_OUT_PIN) == HIGH ? LOW : HIGH);
  delay(5);

  if (Green < Blue && Blue < Red)       //Green
    digitalWrite(LED_PIN3, LOW), digitalWrite(LED_PIN4, HIGH), digitalWrite(LED_PIN5, LOW),colorDetected = 1;
  else if (Red < Green && Green < Blue)  //Yellow
    digitalWrite(LED_PIN3, HIGH), digitalWrite(LED_PIN4, LOW), digitalWrite(LED_PIN5, LOW),colorDetected = 2;
  else if (Blue < Green && Green < Red) //Blue
    digitalWrite(LED_PIN3, LOW), digitalWrite(LED_PIN4, LOW), digitalWrite(LED_PIN5, HIGH),colorDetected = 3;
  else if((Blue < Red && Red < Green) || (Red < Blue && Blue < Green))    //Floor
    digitalWrite(LED_PIN3, LOW), digitalWrite(LED_PIN4, LOW), digitalWrite(LED_PIN5, LOW),colorDetected = 4;
  else                                  //Unknown
    digitalWrite(LED_PIN3, LOW), digitalWrite(LED_PIN4, LOW), digitalWrite(LED_PIN5, LOW),colorDetected = 0;
    
  Serial.print("R:");
  Serial.print(Red);
  Serial.print(" G:");
  Serial.print(Green);
  Serial.print(" B:");
  Serial.print(Blue);
  Serial.print(" ");
  Serial.print(colorDetected);
  Serial.println();
}

//Gets the data from the ultrasonic sensors
void getUltrasonic()
{
  frontUltrasonicDistance[radioNumber] = frontSensor.ping_cm(); //Initiates the front sensor and gets the distance.
  if (frontUltrasonicDistance[radioNumber] < 80 && frontUltrasonicDistance[radioNumber] != 0)
    ultHitFront[radioNumber] = 0;
  else
    ultHitFront[radioNumber] = 1;
 
  if (lane[radioNumber] == 0)
  {
    rightUltrasonicDistance[radioNumber] = 22;//rightSensor.ping_cm(); //Initiates the right sensor and gets the distance.
    if (rightUltrasonicDistance[radioNumber] < 40 && rightUltrasonicDistance[radioNumber] != 0)
      ultHitRight[radioNumber] = 0;
    else
      ultHitRight[radioNumber] = 1; 
  }
  else
  {
    leftUltrasonicDistance[radioNumber] = 22;//leftSensor.ping_cm(); //Initiates the left sensor and gets the distance.
    if (leftUltrasonicDistance[radioNumber] < 80 && leftUltrasonicDistance[radioNumber] != 0)
      ultHitLeft[radioNumber] = 0;
    else
      ultHitLeft[radioNumber] = 1;
  }
  
  if(ultHitFront[radioNumber] | ultHitLeft[radioNumber] |  ultHitRight[radioNumber]) 
  {
    radioNumber_Lane_Ultrasonic[radioNumber] = ( (uint8_t)ultHitFront[radioNumber] | ((uint8_t)ultHitBack[radioNumber] << 1) | ((uint8_t)ultHitRight[radioNumber] << 2) | ((uint8_t)ultHitLeft[radioNumber] << 3) | ((uint8_t)lane[radioNumber] << 4) | radioNumber << 5);
    SendDataConversion();
    role=true;                      
    roleText[radioNumber]=16;
    dataEx();
  }

  Serial.print(frontUltrasonicDistance[radioNumber]);
  //Serial.print(ultHitLeft[radioNumber]);
  //Serial.print(ultHitBack[radioNumber]);
  //Serial.print(ultHitRight[radioNumber]);
  Serial.println();
}

//Determine other cars location
void OtherCarLocation()
{
  if (lane[radioNumber] == lane[!radioNumber])
  {
    if (distance[radioNumber] < distance[!radioNumber])
    {
      otherCarDirection = 0;
    }
    else if (distance[radioNumber] > distance[!radioNumber])
    {
      otherCarDirection = 4;
    }
    else
    {
      otherCarDirection = 4;
    }
  }
  else if (lane[radioNumber] == 0)
  {
    if (distance[radioNumber] < distance[!radioNumber])
    {
      otherCarDirection = 7;
    }
    else if (distance[radioNumber] > distance[!radioNumber])
    {
      otherCarDirection = 5;
    }
    else
    {
      otherCarDirection = 6;
    }
  }
  else if (lane[radioNumber] == 1)
  {
    if (distance[radioNumber] < distance[!radioNumber])
    {
      otherCarDirection = 1;
    }
    else if (distance[radioNumber] > distance[!radioNumber])
    {
      otherCarDirection = 3;
    }
    else
    {
      otherCarDirection = 2;
    }
  }
}

// this function pull the desired bits
int pullWantedBits(uint64_t value, int most_significant, int less_significant)
{
  int result = value >> less_significant;
  result = result % ( 1 << ( most_significant - less_significant + 1 ) );
  return result;
}

// make possible to print on serial monitor a 64 bit data
size_t print64(Print* pr, uint64_t n) 
{
  char buf[21];
  char *str = &buf[sizeof(buf) - 1];
  *str = '\0';
  do 
  {
    uint64_t m = n;
    n /= 10;
    *--str = m - 10*n + '0';
  } while (n);
  pr->print(str);
}

size_t println64(Print* pr, uint64_t n) 
{
  return print64(pr, n) + pr->println();
}

void LED_off()
{
  digitalWrite(LED_PIN4, LOW);
  digitalWrite(LED_PIN5, LOW);
  digitalWrite(LED_PIN6, LOW);
}


//----------------------------------------------------------------------------------------********************************************************
void modeRx()
{
  if (role)
  {
  role = false; roleText[radioNumber]=17;
  csCarInfo(info[13], radioNumber, info[14], lane[radioNumber], info[15], direction[radioNumber], info[12], distance[radioNumber], info[usNumber[radioNumber]], frontUltrasonicDistance[radioNumber],info[roleText[radioNumber]]);
  }
}

void remoteSerial()
{

  if (Serial.available()) // change the role via the serial monitor
  {

    char c = Serial.read();
    if (c == 's') 
    {
      role=true;
      payload[radioNumber] = 0;
      Serial.println("stop");
      dataEx();
    } 
    else if (c == 'f') 
    {
      role=true;
      payload[radioNumber] = 1;
      Serial.println("forward");
      dataEx();
    }
    else if (c == 'b') 
    {
      role=true;
      payload[radioNumber] = 2;
      Serial.println("back");
      dataEx();
    }
    else if (c == 'r') 
    {
      role=true;
      payload[radioNumber] = 3;
      Serial.println("right");
      dataEx();
    }
    else if (c == 'l') 
    {
      role=true;
      payload[radioNumber] = 4;
      Serial.println("left");
      dataEx();
    }
  } 

}

void ultrasonictest()
{
  frontUltrasonicDistance[radioNumber] = frontSensor.ping_cm(); //Initiates the front sensor and gets the distance.
  backUltrasonicDistance[radioNumber] = backSensor.ping_cm();
  leftUltrasonicDistance[radioNumber] = leftSensor.ping_cm();
  rightUltrasonicDistance[radioNumber] = rightSensor.ping_cm();
    if (frontUltrasonicDistance[radioNumber] < 80 && frontUltrasonicDistance[radioNumber] != 0)
    {
      Serial.println(frontUltrasonicDistance[radioNumber]);
      ultHitFront[radioNumber] = 1;
      radioNumber_Lane_Ultrasonic[radioNumber] = ((uint8_t)radioNumber << 5) | ((uint8_t)lane[radioNumber] << 4) | ((uint8_t)ultHitRight[radioNumber] << 3) | ((uint8_t)ultHitBack[radioNumber] << 2) | ((uint64_t)ultHitLeft[radioNumber] << 1) | ((uint8_t)ultHitFront[radioNumber]);
      SendDataConversion();
      csCarInfo(info[13], radioNumber, info[14], lane[radioNumber], info[15], direction[radioNumber], info[12], distance[radioNumber], info[usNumber[radioNumber]], frontUltrasonicDistance[radioNumber],info[roleText[radioNumber]]);
      role = true; 
      roleText[radioNumber]=16;
      dataEx();
      delay(10);
     //priority = 0;
      laneChange_old();
    }
    else
    {
        ultHitFront[radioNumber] = 0;
    }
}

void ultrasonictest_laz()
{

  frontUltrasonicDistance[radioNumber] = frontSensor.ping_cm(); //Initiates the front sensor and gets the distance.
  backUltrasonicDistance[radioNumber] = 22;
  leftUltrasonicDistance[radioNumber] = 22;
  rightUltrasonicDistance[radioNumber] = 22;
    if (frontUltrasonicDistance[radioNumber] < 80 && frontUltrasonicDistance[radioNumber] != 0)
    {
      Serial.println(frontUltrasonicDistance[radioNumber]);
      ultHitFront[radioNumber] = 1;
      //motor.write(80);
      SendDataConversion();
      role = true; roleText[radioNumber]=16;
      csCarInfo(info[13], radioNumber, info[14], lane[radioNumber], info[15], direction[radioNumber], info[12], distance[radioNumber], info[usNumber[radioNumber]], frontUltrasonicDistance[radioNumber],info[roleText[radioNumber]]);
      dataEx();
      laneChange_old();
      radioNumber_Lane_Ultrasonic[radioNumber] = ((uint8_t)ultHitRight[radioNumber] << 3) | ((uint8_t)ultHitBack[radioNumber] << 2) | ((uint64_t)ultHitLeft[radioNumber] << 1) | ((uint8_t)ultHitFront[radioNumber]);

    }
    else
    {
        ultHitFront[radioNumber] = 0;
    }

}

void control()
{ 
  if (lane[radioNumber] != lane[tempRadioNumber])
  {
    if (frontUltrasonicDistance[tempRadioNumber] < 80)//adjust speed for lane change
    {
      if (distance[radioNumber] == distance[tempRadioNumber])
      {
        motor.write(carSpeedFast[radioNumber]);
        delay(500);
        motor.write(carSpeedSlow[radioNumber]);
      }

      if (distance[radioNumber] > distance[tempRadioNumber])
      {
        motor.write(carSpeedSlow[radioNumber]);
      }

      if (distance[radioNumber] < distance[tempRadioNumber])
      {
        motor.write(carSpeedSlow[radioNumber]);
      }
  }
}

  /*
  if (tpayload == 1)//drive forward
  {
    //motor.write(91);
  }
  if (tpayload == 2)//drive back
  {
    //motor.write(60);
  }
  if (tpayload == 3 )//drive right
  {
    //steer.write(carServoRight[radioNumber]);
  }
  if (tpayload == 4)//drive left
  {
    //steer.write(carServoLeft[radioNumber]);
  }
  */
}

void drive()
{
  getUltrasonic();
  getColor();
  
  if (distance[radioNumber] < 3 && laneChangeCount < 1)
  {
    lineFollow();
  }
  else if (laneChangeCount <= 20)
  {
    if (colorDetected == 1 && distanceLock == 0)
    {
      distance[radioNumber]++;
      distanceLock = 1;
    }
    else if(colorDetected == 4) //Floor
    {
      distanceLock = 0;
    }
    laneChange_old();
  }
  else
  {
    distance[radioNumber] = 0;
    lane[radioNumber] = !lane[radioNumber];
    laneChangeCount = 0;
  }
  csCarInfo(info[13], radioNumber, info[14], lane[radioNumber], info[15], direction[radioNumber], info[12], distance[radioNumber], info[usNumber[radioNumber]], frontUltrasonicDistance[radioNumber],info[roleText[radioNumber]]);
  steer.write(steerValue);
  delay(50);
  cycleCount++;
}

// Function to read Red Pulse Widths (combine these pw functions??????????)
int getRedPW()
{

  digitalWrite(COLOR_S2_PIN,LOW);
  digitalWrite(COLOR_S3_PIN,LOW);
  delay(5);
  PW = pulseIn(COLOR_OUT_PIN, LOW);
  return PW;

}

// Function to read Green Pulse Widths
int getGreenPW()
{

  digitalWrite(COLOR_S2_PIN,HIGH);
  digitalWrite(COLOR_S3_PIN,HIGH);
  delay(5);
  PW = pulseIn(COLOR_OUT_PIN, LOW);
  return PW;

}

// Function to read Blue Pulse Widths
int getBluePW()
{

  digitalWrite(COLOR_S2_PIN,LOW);
  digitalWrite(COLOR_S3_PIN,HIGH);
  delay(5);
  PW = pulseIn(COLOR_OUT_PIN, LOW);
  return PW;

}

//Function to read/write color-----------------------
//Function to read/write color-----------------------
void getColor_old()
{

  redPW = getRedPW();
  redValue = map(redPW, redMin,redMax,0,255);
  greenPW = getGreenPW();
  greenValue = map(greenPW, greenMin,greenMax,0,255);
  bluePW = getBluePW();
  blueValue = map(bluePW, blueMin,blueMax,0,255);
  
}
//Function to read/write color-------------------------  

//function to set max/min color values-----------------
void colorCheck(){
  colorCheckRed = 0;
  colorCheckGreen = 0;
  colorCheckBlue =0;
  //White---------------------------------
  cs(info[1], color[0], 0, 0, 0);Serial.print("Ready White\n");
  delay(A);
  cs(info[0], info[2], 0, 0, 0);Serial.print("Measuring\n");
  for(int i=0;i<colorAccuracy;i++){    
    delay(B);
    getColor_old();
    if  (greenPW < greenMin)  
      greenMin = greenPW;        
    else if  (bluePW < blueMin)  
      blueMin = bluePW;
    else if  (redPW < redMin)  
      redMin = redPW; 
  }
  csInt(color[0], info[3], redMin, greenMin, blueMin, 0, 0, 0);
  delay(A);
  Serial.println("White set");
  motor.write(90); delay(colorScanDelay-150);  motor.write(80);
  //White---------------------------------

  //Black---------------------------------
  cs(info[1], color[1], 0, 0, 0);Serial.print("Ready Black\n");
  delay(A);
  cs(info[0], info[2], 0, 0, 0);Serial.print("Measuring\n");
  for(int i=0;i<colorAccuracy;i++){
    delay(B);
    getColor_old();
    if (greenPW > greenMax )         
      greenMax = greenPW;                                    
    else if (bluePW > blueMax )         
      blueMax = bluePW;                             
    else if(redPW > redMax )         
      redMax = redPW;                        
    }
      csInt(color[1], info[3], redMax, greenMax, blueMax, 0, 0, 0);
      delay(A);
      Serial.println("Black set");
      motor.write(90); delay(colorScanDelay);  motor.write(80);
  //Black---------------------------------

  //Track Red--------------------------------
  cs(info[1], color[2], 0, 0, 0);Serial.print("Ready Red\n");
  delay(A);
  cs(info[0], info[2], 0, 0, 0); Serial.print("Measuring\n");
  for(int i=0;i<colorAccuracy;i++){
    delay(B);
    getColor_old();
    if (greenValue < gtrmin )         
      gtrmin = greenValue;                     
    else if  (greenValue > gtrmax)  
      gtrmax = greenValue;        
    else if (blueValue < btrmin )         
       btrmin = blueValue;                     
    else if  (blueValue >btrmax)  
      btrmax = blueValue;
    else if(redValue < rtrmin )         
      rtrmin = redValue;                        
    else if  (redValue > rtrmax)  
      rtrmax = redValue; 
  }
  rtrmin=rtrmin-colorSpread;
  rtrmax=rtrmax+colorSpread;
  gtrmin=gtrmin-colorSpread;
  gtrmax=gtrmax+colorSpread;
  btrmin=btrmin-colorSpread;
  btrmax=btrmax+colorSpread;
  csInt(color[2], info[3], rtrmin, rtrmax, gtrmin, gtrmax, btrmin, btrmax);
  delay(A);
  Serial.println("Red set");
  motor.write(90); delay(colorScanDelay+200);  motor.write(80);
  //Track Red--------------------------------

  //Track Green--------------------------------
  cs(info[1], color[3], 0, 0, 0);Serial.print("Ready Green\n");
  delay(A);
  cs(info[0], info[2], 0, 0, 0); Serial.print("Measuring\n");
  for(int i=0;i<colorAccuracy;i++){
    delay(B);
    getColor_old();
    if (greenValue < gtgmin )         
      gtgmin = greenValue;                     
    else if  (greenValue > gtgmax)  
      gtgmax = greenValue;        
    else if (blueValue < btgmin )         
      btgmin = blueValue;                     
    else if  (blueValue >btgmax)  
      btgmax = blueValue;
    else if(redValue < rtgmin )         
      rtgmin = redValue;                        
    else if  (redValue > rtgmax)  
      rtgmax = redValue; 
  }
  rtgmin=rtgmin-colorSpread;
  rtgmax=rtgmax+colorSpread;
  gtgmin=gtgmin-colorSpread;
  gtgmax=gtgmax+colorSpread;
  btgmin=btgmin-colorSpread;
  btgmax=btgmax+colorSpread;
  csInt(color[3], info[3], rtgmin, rtgmax, gtgmin, gtgmax, btgmin, btgmax);
  delay(A);
  Serial.println("Green set");
  motor.write(90); delay(colorScanDelay+200);  motor.write(80);
  //Track Green--------------------------------

  //Track Blue--------------------------------
  cs(info[1], color[4], 0, 0, 0);Serial.print("Ready Blue\n");
  delay(A);
  cs(info[0], info[2], 0, 0, 0); Serial.print("Measuring\n");
  for(int i=0;i<colorAccuracy;i++){
    delay(B);
    getColor_old();
    if (greenValue < gtbmin )         
      gtbmin = greenValue;                     
    else if  (greenValue > gtbmax)  
      gtbmax = greenValue;
    else if (blueValue < btbmin )         
      btbmin = blueValue;                     
    else if  (blueValue >btbmax)  
      btbmax = blueValue;
    else if(redValue < rtbmin )         
      rtbmin = redValue;                        
    else if  (redValue > rtbmax)  
      rtbmax = redValue; 
  }
  rtbmin=rtbmin-colorSpread;
  rtbmax=rtbmax+colorSpread;
  gtbmin=gtbmin-colorSpread;
  gtbmax=gtbmax+colorSpread;
  btbmin=btbmin-colorSpread;
  btbmax=btbmax+colorSpread;
  csInt(color[4], info[3], rtbmin, rtbmax, gtbmin, gtbmax, btbmin, btbmax);
  delay(A);
  Serial.println("Blue set");
  motor.write(90); delay(colorScanDelay+600);  motor.write(80);
  //Track Blue--------------------------------

  //Track floor--------------------------------
  cs(info[1], color[10], 0, 0, 0);Serial.print("Ready Floor\n");
  delay(A);
  cs(info[0], info[2], 0, 0, 0); Serial.print("Measuring\n");
  for(int i=0;i<colorAccuracy;i++){
    delay(B);
    getColor_old();
    if (greenValue < gtfmin )         
      gtfmin = greenValue;                     
    else if  (greenValue > gtfmax)  
      gtfmax = greenValue;
    else if (blueValue < btfmin )         
      btfmin = blueValue;                     
    else if  (blueValue >btfmax)  
      btfmax = blueValue;
    else if(redValue < rtfmin )         
      rtfmin = redValue;                        
    else if  (redValue > rtfmax)  
      rtfmax = redValue; 
  }
  rtfmin=rtfmin-colorSpread;
  rtfmax=rtfmax+colorSpread;
  gtfmin=gtfmin-colorSpread;
  gtfmax=gtfmax+colorSpread;
  btfmin=btfmin-colorSpread;
  btfmax=btfmax+colorSpread;
  csInt(color[10], info[3], rtfmin, rtfmax, gtfmin, gtfmax, btfmin, btfmax);
  delay(A);
  Serial.println("Floor set");
  motor.write(90); delay(colorScanDelay);  motor.write(80);
  //Track floor--------------------------------
}
//function to set max/min color values----------------------- 

//function to eval-------------------------------------------
//function to eval-------------------------------------------
void evalColor()
{
  getColor_old();
  delay(2);
  if  (redValue >=rtrmin && redValue <=rtrmax && greenValue >= gtrmin && greenValue <= gtrmax && blueValue >= btrmin && blueValue <= btrmax)
  {
    red();
    colorDetected=2;//red
  }
  if  (redValue >=rtgmin && redValue <=rtgmax && greenValue >= gtgmin && greenValue <= gtgmax && blueValue >= btgmin && blueValue <= btgmax) 
  { 
    green();
    colorDetected=1;//green
  }
  if (redValue >=rtbmin && redValue <=rtbmax && greenValue >= gtbmin && greenValue <= gtbmax && blueValue >= btbmin && blueValue <= btbmax)
  { 
    blue();
    colorDetected=3;//blue
  }
  if (redValue >=rtfmin && redValue <=rtfmax && greenValue >= gtfmin && greenValue <= gtfmax && blueValue >= btfmin && blueValue <= btfmax)
  { 
    LED_off();
    colorDetected=4;//floor
  }
  /*
  else
  {
    LED_off();
    colorDetected=4;//unknown 
  }
  */
}
//function to eval-------------------------------------------

//function to Choose car on LCD
void csCar(String firstRow1, String firstRow2, String firstRow3, String firstRow4, String firstRow5, String secondRow1, int secondRowInt1)
{
  lcd.clear();  //Clear the LCD
  lcd.setCursor(0, 0); //Set cursor to the 1st row
  lcd.print(firstRow1); lcd.print(" "); lcd.print(firstRow2); lcd.print(" ");lcd.print(firstRow3); lcd.print(" ");lcd.print(firstRow4); lcd.print(" "); lcd.print(firstRow5);
  lcd.setCursor(0, 1); //Set cursor to the 2nd row
  lcd.print(secondRow1); lcd.print(" "); lcd.print(secondRowInt1);
}
//function to Choose car on  LCD

void csCarInfo(String firstRow1, int firstRowInt1, String firstRow2, int firstRowInt2, String firstRow3, int firstRowInt3, String secondRow1, int secondRowInt1, String secondRow2, int secondRowInt2, String secondRow3)
{
  lcd.clear();  //Clear the LCD
  lcd.setCursor(0, 0); //Set cursor to the 1st row
  lcd.print(firstRow1); lcd.print(firstRowInt1); lcd.print(" "); 
  lcd.print(firstRow2); lcd.print(firstRowInt2); lcd.print(" ");
  lcd.print(firstRow3); lcd.print(firstRowInt3); lcd.print(" ");
  //lcd.print(firstRow4); lcd.print(firstRowInt4); lcd.print(" ");
  lcd.setCursor(0, 1); //Set cursor to the 2nd row
  lcd.print(secondRow1); lcd.print(secondRowInt1); lcd.print(" ");
  lcd.print(secondRow2); lcd.print(secondRowInt2); lcd.print(" ");
  lcd.print(secondRow3); //lcd.print(secondRowInt3); lcd.print(" ");
  //lcd.print(secondRow4); lcd.print(secondRowInt4); lcd.print(" ");
}

//function to print status to LCD
void cs(String firstRow1, String firstRow2, int secondRowInt1, int secondRowInt2, int secondRowInt3)
{
  lcd.clear();  //Clear the LCD
  lcd.setCursor(0, 0); //Set cursor to the 1st row
  lcd.print(firstRow1); lcd.print(" "); lcd.print(firstRow2);
  lcd.setCursor(0, 1); //Set cursor to the 2nd row
  lcd.print(secondRowInt1);
  lcd.setCursor(5, 1); //Set cursor to the 2nd row 5th value
  lcd.print(secondRowInt2);
  lcd.setCursor(10, 1); //Set cursor to the 2nd row 10th value
  lcd.print(secondRowInt3);
}

//function to print #s to LCD
void csInt(String firstRow1, String firstRow2, int secondRowInt1, int secondRowInt2, int secondRowInt3, int secondRowInt4, int secondRowInt5, int secondRowInt6)
{
  lcd.clear();  //Clear the LCD
  lcd.setCursor(0, 0); //Set cursor to the 1st row
  lcd.print(firstRow1); lcd.print(" "); lcd.print(firstRow2);
  lcd.setCursor(0, 1); //Set cursor to the 2nd row
  lcd.print(secondRowInt1); lcd.print(" ");
  lcd.print(secondRowInt2); lcd.print(" ");
  lcd.print(secondRowInt3); lcd.print(" ");
  lcd.print(secondRowInt4); lcd.print(" ");
  lcd.print(secondRowInt5); lcd.print(" ");
  lcd.print(secondRowInt6); lcd.print(" ");
}
//function to print #s to LCD

//function to pulse ultrasonic front
void ultrasonicFront()
{

  digitalWrite(TRIG_PIN1, LOW);
  delay(2);
  digitalWrite(TRIG_PIN1, HIGH);
  delay(10);
  digitalWrite(TRIG_PIN1, LOW);

  durationFront = pulseIn(ECHO_PIN1, HIGH);
  distanceFront = (durationFront*.0343)/2;
  Serial.print("Distance Front:");
  Serial.println(distanceFront);

}
//function to pulse ultrasonic front

//function to pulse ultrasonic left
void ultrasonicLeft()
{

  digitalWrite(TRIG_PIN2, LOW);
  delay(2);
  digitalWrite(TRIG_PIN2, HIGH);
  delay(10);
  digitalWrite(TRIG_PIN2, LOW);

  durationLeft = pulseIn(ECHO_PIN2, HIGH);
  distanceLeft = (durationLeft*.0343)/2;
  Serial.print("Distance Left:");
  Serial.println(distanceLeft);

}
//function to pulse ultrasonic left

//function to pulse ultrasonic back
void ultrasonicBack()
{

  digitalWrite(TRIG_PIN3, LOW);
  delay(2);
  digitalWrite(TRIG_PIN3, HIGH);
  delay(10);
  digitalWrite(TRIG_PIN3, LOW);

  durationBack = pulseIn(ECHO_PIN3, HIGH);
  distanceBack = (durationBack*.0343)/2;
  Serial.print("Distance Back:");
  Serial.println(distanceBack);

}
//function to pulse ultrasonic back

//function to pulse ultrasonic right
void ultrasonicRight()
{

  digitalWrite(TRIG_PIN4, LOW);
  delay(2);
  digitalWrite(TRIG_PIN4, HIGH);
  delay(10);
  digitalWrite(TRIG_PIN4, LOW);

  durationRight = pulseIn(ECHO_PIN4, HIGH);
  distanceRight = (durationRight*.0343)/2;
  Serial.print("Distance Back:");
  Serial.println(distanceRight);

}
//function to pulse ultrasonic front

//function to pulse ultrasonic front,left,back,right
void getUltra()
{

  ultrasonicFront();
  ultrasonicLeft();
  ultrasonicBack();
  ultrasonicRight();
  
}
//function to pulse ultrasonic front,left,back,right

//function to correct to center lane from right marker
void driveCenterLeft()
{
   
  motor.write(speedOneQuarter);
  steer.write(correctLeft);
  delay(correctTime); 
  steer.write(correctRight);
  delay(correctTime);
  steer.write(straight);
  delay(correctTime);
     
}
//function to correct to center lane from right marker

//function to correct to center lane from left marker
void driveCenterRight()
{

  steer.write(correctRight);
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval[radioNumber]) 
  {
    previousMillis = currentMillis;
  }  
  motor.write(speedOneQuarter);
  steer.write(correctRight);
  delay(correctTime); 
  steer.write(correctLeft);
  delay(correctTime);
  steer.write(straight);
  delay(correctTime);

}
//function to correct to center lane from left marker

//function main to drive

void drive_old()
{

  if (colorDetected==distanceMarker && disColor==0)
  {
    distance[radioNumber]++;
    disColor = 1;
    //csCar(info[11],color[9],info[11],color[9],info[11],info[6],info[8],carNumber,info[12],distance);
  }
  if (colorDetected!=distanceMarker)
  {
    disColor=0;
  }
  if(colorDetected==rightLaneMarker)
  {
    steer.write(correctRight);
    currentMillis = millis();
  }
  if (currentMillis - previousMillis >= interval[radioNumber]) 
  {
    steer.write(correctLeft);
    previousMillis = currentMillis;
  }  
  if (currentMillis - previousMillis >= interval[radioNumber])
  {
    steer.write(straight);
    previousMillis = currentMillis;
  }

  if (colorDetected==leftLaneMarker)
  {
    driveCenterRight();
  }
  else
  {
    motor.write(speedOneQuarter);
  }
  
}

//function main to drive

//function to lane change

void laneChange_old()
{

  frontUltrasonicDistance[radioNumber] = 100;
  if (lane[radioNumber] == 0)
  {
    
    steer.write(carServoRightMax[radioNumber]);
    previousMillis = millis();
    
    while(laneLock == 0)
    {
      
      evalColor();
      modeRx();
      dataEx();
      if (colorDetected == 1)
      {
        green();
        if (distanceLock == 0)
        {
          distance[radioNumber]++;
          csCarInfo(info[13], radioNumber, info[14], lane[radioNumber], info[15], direction[radioNumber], info[12], distance[radioNumber], info[usNumber[radioNumber]], frontUltrasonicDistance[radioNumber],info[roleText[radioNumber]]);
          distanceLock = 1;
        }
      }  
      currentMillis = millis();  
      if ((currentMillis - previousMillis) >= interval[radioNumber]) 
      {
        steer.write(carServoLeft[radioNumber]);
        delay(300); 
        lane[radioNumber] = 1;
        csCarInfo(info[13], radioNumber, info[14], lane[radioNumber], info[15], direction[radioNumber], info[12], distance[radioNumber], info[usNumber[radioNumber]], frontUltrasonicDistance[radioNumber],info[roleText[radioNumber]]);  
        laneLock = 1 ;
      }
      
    if (colorDetected == 2){distanceLock = 0;}
    if (colorDetected == 3){distanceLock = 0;}
    if (colorDetected == 4){distanceLock = 0;}
    }

  laneLock = 0;
  }

  else if (lane[radioNumber] == 1)
  {
    frontUltrasonicDistance[radioNumber] = 100;
    steer.write(carServoLeftMax[radioNumber]);
    previousMillis = millis();
    while(laneLock == 0)
    {
      evalColor();
      if (colorDetected == 1)
      {
        green();
        if (distanceLock == 0)
        {
          distance[radioNumber]++;
          csCarInfo(info[13], radioNumber, info[14], lane[radioNumber], info[15], direction[radioNumber], info[12], distance[radioNumber], info[usNumber[radioNumber]], frontUltrasonicDistance[radioNumber],info[roleText[radioNumber]]);
          distanceLock = 1;
        }
      }
      currentMillis = millis();  
      if ((currentMillis - previousMillis) >= interval[radioNumber]) 
      {
        steer.write(carServoRightMax[radioNumber]);
        delay(200);
        lane[radioNumber] = 0;
        csCarInfo(info[13], radioNumber, info[14], lane[radioNumber], info[15], direction[radioNumber], info[12], distance[radioNumber], info[usNumber[radioNumber]], frontUltrasonicDistance[radioNumber],info[roleText[radioNumber]]);  
        laneLock=1;
        /*
        if (laneLock == 0)
        {
        lane[radioNumber] = 0;
        csCarInfo(info[13], radioNumber, info[14], lane[radioNumber], info[15], direction[radioNumber], info[12], distance[radioNumber], info[usNumber[radioNumber]], frontUltrasonicDistance[radioNumber],info[roleText[radioNumber]]);  
        laneLock=1;
        }
        */
      }

      if (colorDetected == 2){distanceLock = 0;}
      if (colorDetected == 3){distanceLock = 0;}
      if (colorDetected == 4){distanceLock = 0;}
    }
  laneLock = 0;
  /*
  while (colorDetected != 2)
  {
    
  }
  */
  }
}

void dataEx()
{

  if (role) // This device is a TX node
  {
    sendData();
  } 
  else // This device is a RX node
  {
    receiveData();
  }

}

void sendData()
{
radio.stopListening();
  unsigned long start_timer = micros();                    // start the timer
  bool report = radio.write(&payload[radioNumber], sizeof(uint64_t));      // transmit & save the report
  unsigned long end_timer = micros();  

  if (report) 
  {
    Serial.print(F("Transmission successful! "));          // payload was delivered
    Serial.print(F("Time to transmit = "));
    Serial.print(end_timer - start_timer);                 // print the timer result
    Serial.print(F(" us. Sent: "));
    //Serial.print();
    //println64(&Serial, payload[radioNumber]);                                // print payload sent
  } 
  else 
  {
    Serial.println(F("Transmission failed or timed out")); // payload was not delivered
  }

}

void receiveData_old()
{
    
  radio.startListening();
  uint8_t pipe;
  if (radio.available(&pipe)) // is there a payload? get the pipe number that recieved it
  {             
    uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
    radio.read(&receivedInformation, bytes);            // fetch payload from FIFO
    Serial.print(F("Received "));
    Serial.print(bytes);                    // print the size of the payload
    Serial.print(F(" bytes on pipe "));
    Serial.print(pipe);                     // print the pipe number
    Serial.print(F(": "));
    println64(&Serial, receivedInformation);                // print the payload's value
    tpayload=receivedInformation;
    control();  
    bitmap(); 
    ReceivedDataConversion();//lane,left, right, back, front
    Serial.println("Radio(3 bit), Lane(1 bit), LeftUl(1 bit),RightUl(1 bit), BackUlt(1 bit), Front(1 bit) = ");
    bitmap8(radioNumber_Lane_Ultrasonic[radioNumber]);
    Serial.println();
    Serial.print("Distance = ");
    bitmap8(distance[tempRadioNumber]);
    Serial.println();
    Serial.print("Left ultrasonic distance");
    bitmap8(leftUltrasonicDistance[tempRadioNumber]);
    Serial.print(" = ");
    Serial.println(leftUltrasonicDistance[tempRadioNumber]);
    Serial.println("Right ultrasonic distance");
    bitmap8(rightUltrasonicDistance[tempRadioNumber]);
    Serial.print(" = ");
    Serial.println(rightUltrasonicDistance[tempRadioNumber]);
    Serial.println("Back ultrasonic distance");
    bitmap8(backUltrasonicDistance[tempRadioNumber]);
    Serial.print(" = ");
    Serial.println(backUltrasonicDistance[tempRadioNumber]);
    Serial.println("Front ultrasonic distance");
    bitmap8(frontUltrasonicDistance[tempRadioNumber]);
    Serial.println(" = ");
    Serial.println(frontUltrasonicDistance[tempRadioNumber]);
  }
  
}

void avoid()
{

}

//function to stop if ultrasonic is within distance(more code later)
void evalUltra()
{

  if (distanceFront < 80)
  {
    motor.write(80);
    goDrive=0;
  }

}
//function to stop if ultrasonic is within distance(more code later)

//function
void sensorSweep()
{
  getColor();
  evalColor();
  getUltra();
  evalUltra();
}
//function

void receiveData()
{
  radio.startListening();
  uint8_t pipe;
  if (radio.available(&pipe)) // is there a payload? get the pipe number that recieved it
  {     
  uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
    radio.read(&receivedInformation, bytes);            // fetch payload from FIFO
    Serial.print(F("Received "));
    Serial.print(bytes);                    // print the size of the payload
    Serial.print(F(" bytes on pipe "));
    Serial.print(pipe);  
    Serial.println();   
    Serial.println();              
    // print the pipe number
    Serial.println("------------------------------------------------------------------------------------------------");
    Serial.println("------------------------------------------------------------------------------------------------");
    
    // print the payload's value
    tpayload=receivedInformation;
    Serial.print("The payload in decimal is: ");
    println64(&Serial, receivedInformation);
    Serial.print("The payload in bits is: ");
    bitmap();
    Serial.println();
    Serial.println("------------------------------------------------------------------------------------------------");
    Serial.println("EACH DATA SHOWED INDIVIDUALLY");
    ReceivedDataConversion();//lane,left, right, back, front
    Serial.print("Radio(3 bit), Lane(1 bit), LeftUl(1 bit),RightUl(1 bit), BackUlt(1 bit), Front(1 bit) = ");
    bitmap8(radioNumber_Lane_Ultrasonic[tempRadioNumber]);
    Serial.println();
    Serial.print("Distance = ");
    Serial.print(distance[tempRadioNumber]);
    Serial.println();
    Serial.print("Distance in bits = ");
    bitmap8(distance[tempRadioNumber]);
    Serial.println();
    Serial.println("------------------------------------------------------------------------------------------------");
    Serial.println("ULTRASONIC DATA");
    Serial.print("Left ultrasonic distance in decimal = ");
    Serial.print(leftUltrasonicDistance[tempRadioNumber]);
    Serial.println();
    Serial.print("Left ultrasonic distance in bits = ");
    bitmap8(leftUltrasonicDistance[tempRadioNumber]);
    Serial.println();
    Serial.print("Right ultrasonic distance in decimal = ");
    Serial.print(rightUltrasonicDistance[tempRadioNumber]);
    Serial.println();
    Serial.print("Right ultrasonic distance in bits = ");
    bitmap8(rightUltrasonicDistance[tempRadioNumber]);
    Serial.println();
    Serial.print("Back ultrasonic distance in decimal = ");
    Serial.print(backUltrasonicDistance[tempRadioNumber]);
    Serial.println();
    Serial.print("Back ultrasonic distance in bits = ");
    bitmap8(backUltrasonicDistance[tempRadioNumber]);
    Serial.println();
    Serial.print("Front ultrasonic distance in decimal = ");
    Serial.print(frontUltrasonicDistance[tempRadioNumber]);
    Serial.println();
    Serial.print("Front ultrasonic distance in bits = ");
    bitmap8(frontUltrasonicDistance[tempRadioNumber]);
    Serial.println();
    Serial.println();
    Serial.println();
    control();
  }
}

void bitmap()
{
      for(int i = 63; i >= 0;i--)
      {
        Serial.print(pullWantedBit(receivedInformation, i));
      }
}
void bitmap8(uint8_t data){

      for(int i = 7; i >= 0;i--)
      {
        Serial.print(pullWantedBit(receivedInformation, i));
      }
}

//Convert Received Data
void ReceivedDataConversion()
{
    // bits 47-40 ,    // tempRadioNumber,lane,left, right, back, front
    priority = pullWantedBits(receivedInformation, 48,48);
    tempRadioNumber = pullWantedBits(receivedInformation, 47,45);
    lane[tempRadioNumber] = pullWantedBits(receivedInformation, 44,44);
    ultHitLeft[tempRadioNumber] = pullWantedBits(receivedInformation, 43,43);
    ultHitRight[tempRadioNumber] = pullWantedBits(receivedInformation, 42,42);
    ultHitBack[tempRadioNumber] = pullWantedBits(receivedInformation, 41,41);
    ultHitFront[tempRadioNumber] = pullWantedBits(receivedInformation, 40,40);
    // bits 39-32
    distance[tempRadioNumber] = pullWantedBits(receivedInformation, 38,32);
    // bits 31-24
    leftUltrasonicDistance[tempRadioNumber] = pullWantedBits(receivedInformation, 31,24);
    // bits 23-16
    rightUltrasonicDistance[tempRadioNumber] = pullWantedBits(receivedInformation, 23,16);
    // bits 14-8
    backUltrasonicDistance[tempRadioNumber] = pullWantedBits(receivedInformation, 15,8);
    // bits 7-0
    frontUltrasonicDistance[tempRadioNumber] = pullWantedBits(receivedInformation, 7,0);
}
//Convert sent data
void SendDataConversion()
{
    payload[radioNumber] = ((0 << 63) |(uint64_t)priority << 48) | ((uint64_t)radioNumber_Lane_Ultrasonic << 40) | ((uint64_t)distance[radioNumber] << 32) | ((uint64_t)leftUltrasonicDistance[radioNumber] << 24) | ((uint64_t)rightUltrasonicDistance[radioNumber] << 16) | ((uint64_t)backUltrasonicDistance[radioNumber] << 8) | ((uint64_t)frontUltrasonicDistance[radioNumber]);
    receivedInformation=payload[radioNumber];
}

int pullWantedBit(uint64_t value, int most_significant)
{
  int result = value >> most_significant;
  result = result % ( 1 << ( most_significant - most_significant + 1 ) );
  return result;
}
void lineFollow_noLCD()
{
  if (lane[radioNumber] == 0)
  {
    if (colorDetected == 1)
    {
      green();
      if (distanceLock == 0)
      {
      distance[radioNumber]++;
      csCarInfo(info[13], radioNumber, info[14], lane[radioNumber], info[15], direction[radioNumber], info[12], distance[radioNumber], info[usNumber[radioNumber]], frontUltrasonicDistance[radioNumber],info[roleText[radioNumber]]);
      distanceLock = 1;
      }
    }
    else if (colorDetected == 2)
    {
      red();
      steerValue = carServoRight[radioNumber]; //Steer right
      distanceLock = 0;
    }
    else if(colorDetected == 4) //other color
    {
      LED_off(); 
      steerValue = carServoLeft[radioNumber];
      distanceLock = 0;
    }
    else
    {
      steerValue = carServoStraight[radioNumber];
    }
    
  }
  if (lane[radioNumber] == 1)
  {
    if (colorDetected == 1) //green
    {
      green();
      if (distanceLock == 0)
      {
      distance[radioNumber]++;
      csCarInfo(info[13], radioNumber, info[14], lane[radioNumber], info[15], direction[radioNumber], info[12], distance[radioNumber], info[usNumber[radioNumber]], frontUltrasonicDistance[radioNumber],info[roleText[radioNumber]]);
      distanceLock = 1;
      }
    }
    else if (colorDetected == 3) //Blue
    {
      blue();
      steerValue = carServoLeft[radioNumber];
      distanceLock = 0;
    }
    else if(colorDetected == 4) //floor
    {
      LED_off(); 
      steerValue = carServoRight[radioNumber];
      distanceLock = 0;
    }
    else
    {
      steerValue = carServoStraight[radioNumber];
    }
    
  }
  
  steer.write(steerValue);
}

void red()
{
  digitalWrite(LED_PIN4, HIGH);
  digitalWrite(LED_PIN5, LOW);
  digitalWrite(LED_PIN6, LOW);
}

void green()
{
  digitalWrite(LED_PIN4, LOW);
  digitalWrite(LED_PIN5, HIGH);
  digitalWrite(LED_PIN6, LOW);
}

void blue()
{
  digitalWrite(LED_PIN4, LOW);
  digitalWrite(LED_PIN5, LOW);
  digitalWrite(LED_PIN6, HIGH);
}

void speedTest()
{
  if (button1State==LOW)    //Red button press:
  {
    carSpeedStop[radioNumber]--;
    motor.write(carSpeedStop[radioNumber]);
    csCarInfo(info[13], radioNumber, info[14], lane[radioNumber], info[15], direction[radioNumber], info[12], distance[radioNumber], info[26], carSpeedStop[radioNumber], info[roleText[radioNumber]]);  
  }
  else if(button2State==LOW)//Green button press:
  {
    carSpeedStop[radioNumber]++;
    motor.write(carSpeedStop[radioNumber]);
    csCarInfo(info[13], radioNumber, info[14], lane[radioNumber], info[15], direction[radioNumber], info[12], distance[radioNumber], info[26], carSpeedStop[radioNumber], info[roleText[radioNumber]]);
  }

}

void steerTest()
{
  if (button1State==LOW)    //Red button press:
  {
    carServoStraight[radioNumber]--;
    steer.write(carServoStraight[radioNumber]);
    csCarInfo(info[13], radioNumber, info[14], lane[radioNumber], info[15], direction[radioNumber], info[12], distance[radioNumber], info[27], carServoStraight[radioNumber], info[roleText[radioNumber]]); 
  }
  else if(button2State==LOW)//Green button press:
  {
    carServoStraight[radioNumber]++;
    steer.write(carServoStraight[radioNumber]);
    csCarInfo(info[13], radioNumber, info[14], lane[radioNumber], info[15], direction[radioNumber], info[12], distance[radioNumber], info[27], carServoStraight[radioNumber], info[roleText[radioNumber]]);
  }

}

void commsCheck()
{
  role = true; 
  roleText[radioNumber]=16;
  dataEx();
  delay(10);
}
