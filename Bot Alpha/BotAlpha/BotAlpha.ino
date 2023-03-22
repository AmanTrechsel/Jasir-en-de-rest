/*
 * File name: BotAlpha
 * 
 * Description: This is a program for an Arduino robot called "BotAlpha". It is designed to follow a black line using a PID algorithm while avoiding obstacles that are detected using an echo sensor. 
 * 
 * Authors: Kjeld & Huub
 */

// Libraries
#include <QTRSensors.h> // Library for the reflectance sensors
#include "PinkPanther.h" // Song
#include "LightShow.h" // Led

// Constants - Wheels
const int leftWheelFwd = 11; // Links, vooruit
const int leftWheelBwd = 10; // Links, achteruit
const int rightWheelFwd = 9; // Rechts, achteruit
const int rightWheelBwd = 6; // Rechts, vooruit

// Constants - Rotation Sensors
const byte R1 = 3;  // Rotatiesensor motor rechts
const byte R2 = 2;  // Rotatiesensor motor links

// Constants - CM to steps
const float wheelDiameter = 63.0; // In milimeters
const float stepCount = 20.00; // Steps per revolution

// Constants - Gripper
const int gripper = 5; // Gripper pin 

// Constants - Echo Sensor
const int trigPin = 12;
const int echoPin = 13;

// Variables - Echo Sensor
float duration;
float distance;

int interval = 250; // In ms
unsigned long time_now = 0;

// Variables - Sensor Calibration
bool calibrationComplete = false; // Check if calibration succeeded
bool finish = false; // Check if robot finished

// Sensors
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Variables - PID
const float KP = 0.225; // Proportional value
const float KD = 2.25; // Derivative value
int lastError = 0;

// Base speed of motors
const int M1 = 255;
const int M2 = 255;

// Function to convert cm to steps
int CMtoSteps(float cm) 
{
  int result;  
  
  float circumference = (wheelDiameter * 3.142857) / 10;
  float cmPerStep = circumference / stepCount;  
  
  float endResult = cm / cmPerStep;  
  result = (int) endResult; 
  
  return result;  
}

// Pulse counter
volatile int R1_Count = 0;
volatile int R2_Count = 0;

void setup() 
{
  // Configure - Sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  // led
  setupNeoPixel();
  
  // Configure - Wheels
  pinMode(leftWheelFwd, OUTPUT);
  pinMode(leftWheelBwd, OUTPUT);
  pinMode(rightWheelFwd, OUTPUT);
  pinMode(rightWheelBwd, OUTPUT);

  // Configure - Gripper
  pinMode(gripper, OUTPUT);

  // Configure - Rotation Sensors
  attachInterrupt(digitalPinToInterrupt (R1), ISR_R1_Count, RISING);  // Increase counter A 
  attachInterrupt(digitalPinToInterrupt (R2), ISR_R2_Count, RISING);  // Increase counter B 

  // Configure - Echo Sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Calibrate sensors
  calibrateSensors();
}

// Calibration
void calibrateSensors()
{
  while(!calibrationComplete)
  {
    openGripper();
    moveForward(CMtoSteps(30), 255);
    delay(500);
    closeGripper();
    rotateLeft(8, 255);
    calibrationComplete = true;
  }
}

void loop()
{
    if(!finish)
    {
      PID();
      avoidObjects();
      stopWhenBlack();
      party();
    }
    else
    {
      brake();
      PinkPanther();
    }
}

// PID
void PID()
{
  uint16_t position = qtr.readLineBlack(sensorValues);
  
  int error = position - 3500; // Calculate error based on perfect position
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;  

  int m1Speed = M1 + motorSpeed;
  int m2Speed = M2 - motorSpeed;

  // Setting minimum and maximum speed for motors
  m1Speed = min(max(m1Speed, 0), 255);
  m2Speed = min(max(m2Speed, 0), 255);

  // Letting robot drive on caluclated speeds
  analogWrite(leftWheelFwd, m1Speed);
  analogWrite(rightWheelFwd, m2Speed);

}

// Move specific CM forward
void moveForward(int steps, int mspeed)
{
  R1_Count = 0;  //  Reset pulse counter A
  R2_Count = 0;  //  Reset pulse counter B

  while (steps > R1_Count && steps > R2_Count) 
  {
    qtr.calibrate();

    if (steps > R1_Count) 
    {
      analogWrite(leftWheelFwd, mspeed);
    } 
    else 
    {
      analogWrite(leftWheelFwd, 0);
    }
    if (steps > R2_Count) 
    {
      analogWrite(rightWheelFwd, mspeed - 8); // Small correction, so that the wheels spin at the same speed
    } 
    else 
    {
      analogWrite(rightWheelFwd, 0);
    }
  }

  // Stop when done
  brake();
  
  R1_Count = 0;  //  Reset pulse counter A
  R2_Count = 0;  //  Reset pulse counter B
}

// Move specific CM backwards
void moveReverse(int steps, int mspeed)
{
  R1_Count = 0;  //  Reset pulse counter A
  R2_Count = 0;  //  Reset pulse counter B

  while (steps > R1_Count && steps > R2_Count) 
  {
    if (steps > R1_Count) 
    {
      analogWrite(leftWheelBwd, mspeed);
    } 
    else 
    {
      analogWrite(leftWheelBwd, 0);
    }
    if (steps > R2_Count) 
    {
      analogWrite(rightWheelBwd, mspeed - 10); // Small correction, so that the wheels spin at the same speed
    } 
    else 
    {
      analogWrite(rightWheelBwd, 0);
    }
  }

  // Stop when done
  brake();
  
  R1_Count = 0;  //  Reset pulse counter A
  R2_Count = 0;  //  Reset pulse counter B
}

void brake()
{
  analogWrite(leftWheelFwd, 0);
  analogWrite(leftWheelBwd, 0);
  analogWrite(rightWheelFwd, 0);
  analogWrite(rightWheelBwd, 0);
}

// Rotate 90 degrees to the left
void rotateLeft(int steps, int mspeed)
{
  R1_Count = 0;  //  Reset pulse counter A
  R2_Count = 0;  //  Reset pulse counter B

  while (steps > R1_Count && steps > R2_Count ) 
  {
    if (steps > R1_Count) 
    {
      analogWrite(leftWheelBwd, mspeed);
    } 
    else 
    {
      analogWrite(leftWheelBwd, 0);
    }
    if (steps > R2_Count) 
    {
      analogWrite(rightWheelFwd, mspeed);
    } 
    else 
    {
      analogWrite(rightWheelFwd, 0);
    }
  }

  // Stop when done
  brake();
  
  R1_Count = 0;  //  Reset pulse counter A
  R2_Count = 0;  //  Reset pulse counter B
}

// Rotate to the right to avoid objects
void rotateRight(int steps, int mspeed)
{
  R1_Count = 0;  //  Reset pulse counter A
  R2_Count = 0;  //  Reset pulse counter B

  while (steps > R1_Count) 
  {
    if (steps > R1_Count) 
    {
      analogWrite(leftWheelFwd, mspeed);
      analogWrite(rightWheelFwd, 100);
    } 
    else 
    {
      analogWrite(leftWheelBwd, 0);
    }
  }

  // Stop when done
  brake();
  
  R1_Count = 0;  //  Reset pulse counter A
  R2_Count = 0;  //  Reset pulse counter B
}

// Motor A pulse count
void ISR_R1_Count()  
{
  R1_Count++; // Increment R1 to count pulses
} 

// Motor B pulse count
void ISR_R2_Count()  
{
  R2_Count++; // Increment R2 to count pulses
}

// Stop when all sensors detect black
void stopWhenBlack() {
  if((sensorValues[0] == 1000) && (sensorValues[1] == 1000) && (sensorValues[2] == 1000) && (sensorValues[3] == 1000) && (sensorValues[4] == 1000) && (sensorValues[5] == 1000) && (sensorValues[6] == 1000) && (sensorValues[7] == 1000))
  {
    brake();
    moveForward(CMtoSteps(15), 255);
    delay(500);
    moveReverse(CMtoSteps(18), 255);
    openGripper();
    moveReverse(CMtoSteps(50), 255);
    finish = true;
  }
}

// Method to avoid objects
void avoidObjects()
{
  if(millis() > time_now + interval)
  {
    time_now = millis();
    
    digitalWrite(trigPin, HIGH);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.0343 / 2; // In CM

    // Avoid object
    if (distance <= 20) 
    {
      rotateLeft(10, 255);
      moveForward(CMtoSteps(16), 255);
      rotateRight(50, 255);
    }
  }
}

// Open gripper
void openGripper()
{
  int i;
  for(i = 0; i < 10; i++)
  {
  digitalWrite(gripper, HIGH);
  delayMicroseconds(1600);
  digitalWrite(gripper, LOW);
  delay(20); 
  }
}

// Close gripper
void closeGripper()
{
  int i;
  for(i = 0; i < 10; i++)
  {
  digitalWrite(gripper, HIGH);
  delayMicroseconds(1100);
  digitalWrite(gripper, LOW);
  delay(20); 
  }
}
