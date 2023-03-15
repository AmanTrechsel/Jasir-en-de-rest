// Libraries
#include <QTRSensors.h>

// Constants - Wheels
const int leftWheelFwd = 11; // Links, vooruit
const int leftWheelBwd = 10; // Links, achteruit
const int rightWheelFwd = 9; // Rechts, achteruit
const int rightWheelBwd = 6; // Rechts, vooruit

// Constanten - Rotation Sensors
const byte R1 = 3;  // Rotatiesensor motor rechts
const byte R2 = 2;  // Rotatiesensor motor links

// Constants - CM to steps
const float wheelDiameter = 63.0; // In milimeters
const float stepCount = 20.00;

// Constants - Gripper
const int gripper = 5; // Gripper pin 
const int closedAngle = 140;
const int openedAngle = 225;

// Sensor Calibration
bool calibrationComplete = false;
bool finish = false;

// Sensors
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// PID
const float KP = 0.225;
const float KD = 2.25;
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
    delay(500);  // Wait one second
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
    stopWhenBlack();
  }
  else
  {
    brake();
  }
}

// PID
void PID()
{
  uint16_t position = qtr.readLineBlack(sensorValues);
  
  int error = position - 3500;
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;  

  int m1Speed = M1 + motorSpeed;
  int m2Speed = M2 - motorSpeed;

  m1Speed = min(max(m1Speed, 0), 255);
  m2Speed = min(max(m2Speed, 0), 255);

  analogWrite(leftWheelFwd, m1Speed);
  analogWrite(rightWheelFwd, m2Speed);
}

// Gripper openen
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

// Gripper sluiten
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

// Aantal centimeters vooruit rijden
void moveForward(int steps, int mspeed)
{
  R1_Count = 0;  //  Reset pulse counter A
  R2_Count = 0;  //  Reset pulse counter B

  while (steps > R1_Count && steps > R2_Count) {
    qtr.calibrate();

    if (steps > R1_Count) {
      analogWrite(leftWheelFwd, mspeed);
    } else {
      analogWrite(leftWheelFwd, 0);
    }
    if (steps > R2_Count) {
      analogWrite(rightWheelFwd, mspeed - 10);
    } else {
      analogWrite(rightWheelFwd, 0);
    }
  }

  // Stop when done
  brake();
  
  R1_Count = 0;  //  Reset pulse counter A
  R2_Count = 0;  //  Reset pulse counter B
}

void moveReverse(int steps, int mspeed)
{
  R1_Count = 0;  //  Reset pulse counter A
  R2_Count = 0;  //  Reset pulse counter B

  while (steps > R1_Count && steps > R2_Count) {

    if (steps > R1_Count) {
      analogWrite(leftWheelBwd, mspeed);
    } else {
      analogWrite(leftWheelBwd, 0);
    }
    if (steps > R2_Count) {
      analogWrite(rightWheelBwd, mspeed);
    } else {
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

// 90 graden naar links draaien
void rotateLeft(int steps, int mspeed)
{
  R1_Count = 0;  //  Reset pulse counter A
  R2_Count = 0;  //  Reset pulse counter B

  while (steps > R1_Count && steps > R2_Count ) {

    if (steps > R1_Count) {
      analogWrite(leftWheelBwd, mspeed);
    } else {
      analogWrite(leftWheelBwd, 0);
    }
    if (steps > R2_Count) {
      analogWrite(rightWheelFwd, mspeed);
    } else {
      analogWrite(rightWheelFwd, 0);
    }
  }

  // Stop when done
  brake();
  
  R1_Count = 0;  //  Reset pulse counter A
  R2_Count = 0;  //  Reset pulse counter B
}

// Motor A pulsen tellen
void ISR_R1_Count()  
{
  R1_Count++;  
} 

// Motor B pulsen tellen
void ISR_R2_Count()  
{
  R2_Count++;
}

// Stop when all sensors detect black
void stopWhenBlack() {
  if((sensorValues[0] == 1000) && (sensorValues[1] == 1000) && (sensorValues[2] == 1000) && (sensorValues[3] == 1000) && (sensorValues[4] == 1000) && (sensorValues[5] == 1000) && (sensorValues[6] == 1000) && (sensorValues[7] == 1000))
  {
    brake();
    moveForward(CMtoSteps(15), 255);
    delay(500);
    moveReverse(CMtoSteps(15), 255);
    openGripper();
    moveReverse(CMtoSteps(50), 255);
    finish = true;
  }
}
