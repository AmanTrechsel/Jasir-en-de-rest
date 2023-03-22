#include <FastLED.h>

const int motorA1 = 10; // the first pin that is connected to motor A (the left motor)
const int motorA2 = 9; // the second pin that is connected to motor A (the left motor)
const int motorB1 = 6; // the first pin that is connected to motor B (the right motor)
const int motorB2 = 5; // the second pin that is connected to motor B (the right motor)
const int sensorMotor1 = 3; // rotation sensor for motor B (the right motor)
const int sensorMotor2 = 2; // rotation sensor for motor A (the left motor)
const int scanner = 12; // servo under the echoSensor
const int gripper = 4;
const int trigPin = 7; // the pin that is connected to the trigger of the ultra sonic sensor
const int echoPin = 8; // the pin that is connected to the echo of the ultra sonic sensor
int counter1 = 0;
int counter2 = 0;
int pos = 0;
int speed;
int turnValue;

#define LED_PIN 11
#define NUM_LEDS 4
CRGB leds[NUM_LEDS];

long duration; // the time it takes for the echo to be detected

int distance;
const int minSafeDistance = 25;



void setup() {
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
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(sensorMotor1), count1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(sensorMotor2), count2, CHANGE);

  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
  FastLED.clear();
  FastLED.show();

}

void loop() 
{
  solveMaze();
}

void solveMaze()
{

  miiMusic();
/*
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
        turnValue = 30;
        moveForward();    
      }
      else
      {
        lightsBad();
        brake();
        turnAround();
        counter2 = 0;
        turnValue = 12;
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
      turnValue = 52;
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
    turnValue = 30;
    moveForward();
  }
  */
}


// neo pixel methods //

void lightsGood()
{
    FastLED.setBrightness(255);
    leds[0] = CRGB(0, 255, 0);
    leds[1] = CRGB(0, 255, 0);
    leds[2] = CRGB(0, 255, 0);
    leds[3] = CRGB(0, 255, 0);
    FastLED.show();
}

void lightsRight() 
{
  FastLED.setBrightness(255);
  leds[2] = CRGB(255, 165, 0);
  leds[1] = CRGB(255, 165, 0);
  leds[3] = CRGB(0, 0, 0);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
}

void lightsBad()
{
  FastLED.setBrightness(255);
  leds[0] = CRGB(255, 0, 0);
  leds[1] = CRGB(255, 0, 0);
  leds[2] = CRGB(255, 0, 0);
  leds[3] = CRGB(255, 0, 0);
  FastLED.show();
}

void lightsLeft()
{
  FastLED.setBrightness(255);
  leds[0] = CRGB(255, 165, 0);
  leds[3] = CRGB(255, 165, 0);
  leds[1] = CRGB(0, 0, 0);
  leds[2] = CRGB(0, 0, 0);
  FastLED.show();
}

void lightsFront()
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

void moveForward()
{
  while (counter2 < turnValue)
  {
    speed = 225;
    leftForward();
    rightForward();
  } 
}

void turnAround()
{
  leftTurn();
  brake();
  delay(200);
  rightTurn();
}

void turnLeft()
{
  turnValue = 37;
  while (counter2 < turnValue)
  {
    speed = 200;
    rightForward();
   }
}

void turnRight()
{
  turnValue = 37;
  while (counter1 < turnValue)
  {
    speed = 200;
    leftForward();
  }
}

void rightTurn()
{
  turnValue = 35;
  while (counter2 < turnValue)
  {
    speed = 200;
    rightForward();
  }
}

void leftTurn()
{
  turnValue = 40;
  while (counter1 < turnValue)
  {
    speed = 200;
    leftBackward();
  }
}

void startUp()
{
  turnValue = 1;
  while (counter2 < turnValue)
  {
    speed = 255;
    rightForward();
    leftForward();
  }
}

void rightBrake () {
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, HIGH);
}

void leftBrake () {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, HIGH);
}

void brake() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, HIGH);
}

void leftForward()
{
  analogWrite(motorA1, 0);
  analogWrite(motorA2, speed - 3);
}

void rightForward()
{
  analogWrite(motorB1, 0);
  analogWrite(motorB2, speed);
}

void leftBackward()
{
  analogWrite(motorA1, speed);
  analogWrite(motorA2, 0);
}

void rightBackward()
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
