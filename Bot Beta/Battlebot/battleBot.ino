/************************************
 *         BATTLE BOT BETA          *    
 *         Jesper en Kimmy          *
 ************************************/

// include documents //
#include <FastLED.h> //Library for the led lights
#include <QTRSensors.h> //Library for the line sensor
#include "music2.h" // finish song
#include "music1.h" //start song

//******************************//

// variables for the wheels //

const int motorA1 = 10; // the first pin that is connected to motor A (the left motor)
const int motorA2 = 9; // the second pin that is connected to motor A (the left motor)
const int motorB1 = 6; // the first pin that is connected to motor B (the right motor)
const int motorB2 = 5; // the second pin that is connected to motor B (the right motor)
const int sensorMotor1 = 3; // rotation sensor for motor B (the right motor)
const int sensorMotor2 = 2; // rotation sensor for motor A (the left motor)

//*************************//

// variables for the echosensor //

const int scanner = 12; // servo under the echoSensor
const int trigPin = 7; // the pin that is connected to the trigger of the ultra sonic sensor
const int echoPin = 8; // the pin that is connected to the echo of the ultra sonic sensor
long duration; // the time it takes for the echo to be detected
int distance; // the distance between the echoSensor and an object
const int minSafeDistance = 25; // the minimum safe distance for the battle bot to be from a wall
const int startDistance = 30; // The minimun distance to start the race

//***************************//

// variables for the gripper //

const int gripper = 4; // the pin that is connected to the servo of the gripper
const int gripperOpen = 1650; //how far the gripper can open
const int gripperClose = 1250; // how small the gripper can close

//**************************//

// variables for the counters //

int counter1 = 0; // counter for the right wheel
int counter2 = 0; // counter for the left wheel

//*************************//

// variables for movement //

int speed; // the variable for the speed of the motors
int movementValue; // the variable for the movement amount

//*********************//

// variables for line sensor //

bool calibrationComplete = false; // Check if calibration succeeded
const int calibrationTime = 26; //Time for calibration
const bool shouldCalibrate = true; //Just an extra check if it should calibrate
QTRSensors qtr; // Specify the qtr sensor library
const uint8_t SensorCount = 8; //How much sensors there are
uint16_t sensorValues[SensorCount]; //specify the variable for the sensorvalues

//********************//

// define pins //
#define LED_PIN 11 //our led pin is on pin 11
#define NUM_LEDS 4 // we have 4 neonpixels
CRGB leds[NUM_LEDS]; // specify how many leds there are

//*********************//

void setup() {
  // Define the input/output list //
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(sensorMotor1, INPUT);
  pinMode(sensorMotor2, INPUT);
  pinMode(trigPin, OUTPUT); // sets the trigger pin as an output
  pinMode(echoPin, INPUT); // sets the echo pin as an input
  pinMode(scanner, OUTPUT);
  pinMode(gripper, OUTPUT);

  //*************************//
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(sensorMotor1), count1, CHANGE);// Every time the sensor value is changed than the code will execute
  attachInterrupt(digitalPinToInterrupt(sensorMotor2), count2, CHANGE);
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A6,A0,A1,A2,A3,A4,A5,A7}, SensorCount); // specify every pin for the line sensor

  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_LEDS); // Add the lights
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500); //Define the lights in volt and milliamps
  FastLED.clear();
  FastLED.show(); // turn on the lights when the code for the lights execute
  frontScan(); // Scan the front
  delay(500); // Wait for 0.5 seconds
  openGripper(); // opens the gripper
  delay(400);
  echoSensor(); // read the sensor out
  //startMusic(); // Start the begin music
  while (distance > startDistance) // check if the distance is bigger than the startdistance // if that's true execute the code
  {
    lightsGood();
    delay(100);
    lightsBad();
    delay(100);
    echoSensor();
  }
    brake();
    delay(2000); // wait for 2 seconds so the other bot can go away
      int i;
      for( i = 0; i < calibrationTime; i++) // execute the code till the calibration time is over, he is executing the code inside the brackets 26 times
      {
        goForward();
        qtr.calibrate();
        delay(10);
      }
      brake();
      closeGripper(); // close the gripper
      delay(400);
      goLeft(); //after 0.4 seconds go left
      delay(900); // turn left for 0.95 seconden
      brake(); // stop
      delay(50);
      goForward(); // go forward in to the maze
      delay(950);
      counter1 = 0;
      counter2 = 0;
      // put the counters back to zero for the next funtions

}

void loop() 
{
  solveMaze(); // call the method to solve the maze
}

void solveMaze()
/********************************************************************************************************************************************************************************************** 
 * First the 2 counters is set to zero, after that the bot brakes and put the right lights on to show that he is going to do a right scan. After 0.5 seconds he is reading the sensor out.    *
 * Than an if statement to check if the distance is smaller than the minimun safe distance, if so the lights are set to bad, than he does a front scan.                                       *
 * If not the lights are set to good and he goes to the right to check if there is a free way to go. Than he goes to the second if the distance is smaller than the minimum the lights go     *
 * bad again en he is going to do a left scan and then the same as the right. The last if checks if the distance is greater than the minimum distance, turn the lights on good and go forward *
 * if not the lights go bad again and he is turning around to look for another way to solve the maze.                                                                                         *
 *********************************************************************************************************************************************************************************************/
{
  counter2 = 0;
  counter1 = 0;
  brake();
  lightsRight();
  rightScan();
  delay(500);
  echoSensor();
  if (distance < minSafeDistance)
  {
    lightsBad();
    delay(200);
    lightsFront();
    frontScan();
    delay(500);
    echoSensor();
    if (distance < minSafeDistance)
    {
      lightsBad();
      delay(200);
      lightsLeft();
      leftScan();
      delay(500);
      echoSensor();
      if (distance > minSafeDistance)
      {
        lightsGood();
        brake();
        startUp();
        counter1 = 0;
        turnLeft();
        counter2 = 0;
        movementValue = 30;
        moveForward();    
      }
      else
      {
        lightsBad();
        brake();
        turnAround();
        counter2 = 0;
        movementValue = 12;
        moveForward();
      }
    }
    else
    {
      lightsGood();
      counter2 = 0;
      brake();
      startUp();
      counter2 = 0;
      movementValue = 52;
      moveForward();
    }
  }
  else
  {
    lightsGood();
    brake();
    startUp();
    counter2 = 0;
    turnRight();
    counter2 = 0;
    movementValue = 30;
    moveForward();
  }
  uint16_t position = qtr.readLineBlack(sensorValues);   
  //stop when all sensors detect black    
  if((sensorValues[0] > 700) && (sensorValues[1] > 700) && (sensorValues[2] > 700) && (sensorValues[3] > 700) && (sensorValues[4] > 700) && (sensorValues[5] > 700) && (sensorValues[6] > 700) && (sensorValues[7] > 700))   
  {     
     brake();    
     delay(50);     
     openGripper();  
     brake();
     backUp();
     delay(500);
     brake();
     finishMusic();
    }
}

// neo pixel methods //

void lightsGood() // turn the lights to full to green
{
    FastLED.setBrightness(255);
    leds[0] = CRGB(0, 255, 0);
    leds[1] = CRGB(0, 255, 0);
    leds[2] = CRGB(0, 255, 0);
    leds[3] = CRGB(0, 255, 0);
    FastLED.show();
}

void lightsRight() // turn the right lights on
{
  FastLED.setBrightness(255);
  leds[2] = CRGB(255, 165, 0);
  leds[1] = CRGB(255, 165, 0);
  leds[3] = CRGB(0, 0, 0);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
}

void lightsBad() // turn all lights to red
{
  FastLED.setBrightness(255);
  leds[0] = CRGB(255, 0, 0);
  leds[1] = CRGB(255, 0, 0);
  leds[2] = CRGB(255, 0, 0);
  leds[3] = CRGB(255, 0, 0);
  FastLED.show();
}

void lightsLeft() //turn the left lights on
{
  FastLED.setBrightness(255);
  leds[0] = CRGB(255, 165, 0);
  leds[3] = CRGB(255, 165, 0);
  leds[1] = CRGB(0, 0, 0);
  leds[2] = CRGB(0, 0, 0);
  FastLED.show();
}

void lightsFront() //turn the front lights on
{
  FastLED.setBrightness(255);
  leds[2] = CRGB(255, 165, 0);
  leds[3] = CRGB(255, 165, 0);
  leds[0] = CRGB(0, 0, 0);
  leds[1] = CRGB(0, 0, 0);
  FastLED.show();
}

//**********************//

// movment methods //

void goForward()  // move forward
{
  analogWrite(motorA1, 0); //left backwards
  analogWrite(motorA2, 215); //left forwards
  analogWrite(motorB1, 0); //right backwards
  analogWrite(motorB2, 225); //right forwards
}


void goLeft() // move to the left
{
  analogWrite(motorA1, 0); //left backwards
  analogWrite(motorA2, 0); //left forwards
  analogWrite(motorB1, 0); //right backwards
  analogWrite(motorB2, 225); //right forwards
}

void backUp() //the bot moves backwards
{
  rightBackward();
  leftBackward();
}

void moveForward() // move forward with pulses
{
  while (counter2 < movementValue)
  {
    speed = 225;
    leftForward();
    rightForward();
  } 
}

void turnAround() // turn the bot first to the left and after that go to the right, an full 180 degrees turn
{
  leftTurn();
  brake();
  delay(200);
  rightTurn();
}

void turnLeft() // go left with pulses
{
  movementValue = 37;
  while (counter2 < movementValue)
  {
    speed = 200;
    rightForward();
   }
}

void turnRight() // go right with pulses and go forward left
{
  movementValue = 37;
  while (counter1 < movementValue)
  {
    speed = 200;
    leftForward();
  }
}

void rightTurn() // go right with pulses and go forward right
{
  movementValue = 35;
  while (counter2 < movementValue)
  {
    speed = 200;
    rightForward();
  }
}

void leftTurn() // go left with pulses and go left backwards
{
  movementValue = 40;
  while (counter1 < movementValue)
  {
    speed = 200;
    leftBackward();
  }
}

void startUp() // turn the rightforward and the leftforward together on
{
  movementValue = 1;
  while (counter2 < movementValue)
  {
    speed = 255;
    rightForward();
    leftForward();
  }
}

void rightBrake () { // brake the right wheel
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, HIGH);
}

void leftBrake () { // brake the left wheel
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, HIGH);
}

void brake() { // turn of the wheels
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, HIGH);
}

void leftForward() // left wheel go forward
{
  analogWrite(motorA1, 0);
  analogWrite(motorA2, speed - 3);
}

void rightForward() // right wheel go forward
{
  analogWrite(motorB1, 0);
  analogWrite(motorB2, speed);
}

void leftBackward() // left wheel go backwards
{
  analogWrite(motorA1, speed);
  analogWrite(motorA2, 0);
}

void rightBackward() // right wheel go backwards
{
  analogWrite(motorB1, speed);
  analogWrite(motorB2, 0);
}

//*****************//

// counter methods //

void count1() {
  counter1++;
}

void count2() {
  counter2++;
}

//******************//

// echosensor methods //

void leftScan () 
{
  for (int i=0; i<10; i++)
  {
  digitalWrite(scanner, HIGH);
  delayMicroseconds(2550);
  digitalWrite(scanner, LOW);
  delayMicroseconds(18550);
  }
}

void rightScan () 
{
  for (int i=0; i<10; i++)
  {
    digitalWrite(scanner, HIGH);
    delayMicroseconds(550);
    digitalWrite(scanner, LOW);
    delayMicroseconds(18550);
  }
}

void frontScan () 
{
  for (int i=0; i<10; i++)
  {
    digitalWrite(scanner, HIGH);
    delayMicroseconds(1550);
    digitalWrite(scanner, LOW);
    delayMicroseconds(18550);
  }
}


void echoSensor () {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  Serial.println(distance);
  delay(10);
}

//****************//

// Methods for gripper //

void openGripper()
{
  for (int i=0; i<10; i++)
  {
  digitalWrite(gripper, HIGH);
  delayMicroseconds(gripperOpen);
  digitalWrite(gripper, LOW);
  delayMicroseconds(18550);
  }
}

void closeGripper()
{
  for (int i=0; i<10; i++)
  {
  digitalWrite(gripper, HIGH);
  delayMicroseconds(gripperClose);
  digitalWrite(gripper, LOW);
  delayMicroseconds(18550);
  }
}

//*****************//
