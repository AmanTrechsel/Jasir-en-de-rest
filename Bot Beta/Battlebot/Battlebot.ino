
//#include <FastLED.h>

//#define LED_PIN 11
//#define NUM_LEDS 4

//CRGB leds[NUM_LEDS];

const int motorA1 = 10; // the first pin that is connected to motor A (the left motor)
const int motorA2 = 9; // the second pin that is connected to motor A (the left motor)
const int motorB1 = 6; // the first pin that is connected to motor B (the right motor)
const int motorB2 = 5; // the second pin that is connected to motor B (the right motor)
const int sensorMotor1 = 3; // rotation sensor for motor B (the right motor)
const int sensorMotor2 = 2; // rotation sensor for motor A (the left motor)
const int scanner = 12; // servo under the echoSensor
const int trigPin = 7; // the pin that is connected to the trigger of the ultra sonic sensor
const int echoPin = 8; // the pin that is connected to the echo of the ultra sonic sensor
const int servoRight = 550;
const int servoCenter = 1600;
const int servoLeft = 2550;
int counter1 = 0;
int counter2 = 0;
int pos = 0;
long duration; // the time it takes for the echo to be detected
int distance; // the distance between the sensor and the object


void setup() {
  pinMode(sensorMotor1, INPUT);
  pinMode(sensorMotor2, INPUT);
  pinMode(trigPin, OUTPUT); // sets the trigger pin as an output
  pinMode(echoPin, INPUT); // sets the echo pin as an input
  pinMode(scanner, OUTPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(sensorMotor1), count1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(sensorMotor2), count2, CHANGE);

 // FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_LEDS);
 // FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
 // FastLED.clear();
 // FastLED.show();
}

void loop() {

  //2550 for left
  //1600 for front
  //550 for right
/*
  frontScan();
  echoSensor();
  forward();

  if (distance <= 15) {
    brake();
    leftScan();
    delay(800);
    echoSensor();
    if (distance <= 15) {
      rightScan();
      delay(800);
      echoSensor();
      if (distance <= 15) {
        counter1 = 0;
        counter2 = 0;
        right();
        if (counter2 >= 32) {
          digitalWrite(motorA1, HIGH);
          digitalWrite(motorA2, HIGH);
        }
        if (counter1 >= 32) {
          digitalWrite(motorB1, HIGH);
          digitalWrite(motorB2, HIGH);
        }
      }
      else {
        counter1 = 0;
        counter2 = 0;
        right();
        if (counter2 >= 20) {
          digitalWrite(motorA1, HIGH);
          digitalWrite(motorA2, HIGH);
        }
        if (counter1 >= 20) {
          digitalWrite(motorB1, HIGH);
          digitalWrite(motorB2, HIGH);
        }
      }
    }
    else {
      counter1 = 0;
      counter2 = 0;
      left();
      if (counter2 >= 20) {
          digitalWrite(motorA1, HIGH);
          digitalWrite(motorA2, HIGH);
        }
        if (counter1 >= 20) {
          digitalWrite(motorB1, HIGH);
          digitalWrite(motorB2, HIGH);
        } 
    }
  } */
}

void echoSensor () {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
}

void count1() {
  counter1++;
}

void count2() {
  counter2++;
}

void forward() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

void backward() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void brake() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, HIGH);
}

void leftScan () {
  digitalWrite(scanner, HIGH);
  delayMicroseconds(servoLeft);
  digitalWrite(scanner, LOW);
  delayMicroseconds(18550);
}

void rightScan () {
  digitalWrite(scanner, HIGH);
  delayMicroseconds(servoRight);
  digitalWrite(scanner, LOW);
  delayMicroseconds(18550);
}

void frontScan() {
  digitalWrite(scanner, HIGH);
  delayMicroseconds(servoCenter);
  digitalWrite(scanner, LOW);
  delayMicroseconds(1550);
}

void left() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

void right() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

/*void lightsGood(){
    FastLED.setBrightness(255);
    leds[0] = CRGB(0, 255, 0);
    leds[1] = CRGB(0, 255, 0);
    leds[2] = CRGB(0, 255, 0);
    leds[3] = CRGB(0, 255, 0);
    FastLED.show();
   
}


void lightsNotGood(){
    FastLED.setBrightness(255);
    leds[0] = CRGB(255, 0, 0);
    leds[1] = CRGB(255, 0, 0);
    leds[2] = CRGB(255, 0, 0);
    leds[3] = CRGB(255, 0, 0);
    FastLED.show();
}

void lightsMiddle(){
    leds[0] = CRGB(255, 100, 0);
    leds[1] = CRGB(255, 100, 0);
    leds[2] = CRGB(255, 100, 0);
    leds[3] = CRGB(255, 100, 0);
    FastLED.show();
}*/
