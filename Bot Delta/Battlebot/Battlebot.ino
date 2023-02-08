#include <QTRSensors.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

// Define the data transmit/receive pins in Arduino
#define TxD 2
#define RxD 3

SoftwareSerial mySerial(RxD, TxD); // RX, TX for Bluetooth

// Gripper
const int gripper = 10;
const int closedAngle = 250;
const int openedAngle = 1400;

// Clicker
const int echo = 12; // Speaker
const int trig = 13; // Microphone
long duration;
int distance;

// Motor
const int leftWheelFwd = 11;
const int leftWheelBwd = 6;
const int rightWheelBwd = 5;
const int rightWheelFwd = 3;
//
int loopcount;
int speedforce = 255;
const int rotatespeed = 150;
const int drivespeed = 150;
int readLineblack;

// Sensors
QTRSensors qtr;
const int lineThreshold = 900;
uint16_t sensors[8];
uint16_t sensors_threshold[8];



void setup()
{
  mySerial.begin(9600); // For Bluetooth
  
  // Echo locator
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // Wheels
  pinMode(leftWheelFwd, OUTPUT);
  pinMode(leftWheelBwd, OUTPUT);
  pinMode(rightWheelFwd, OUTPUT);
  pinMode(rightWheelBwd, OUTPUT);

  // Line Sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {A6,A0,A7,A1,A2,A3,A4,A5},8);
  
  // Initiate Serial
  Serial.begin(9600);

  // Sensor Calibration
  int i;
  Serial.println("");
  Serial.print("Calibrating");
  for (i = 0; i < 250; i++)
  {
    driveBreak();
    if (i%10==0)
    {
      i%20==0 ? rotateLeft() : rotateRight();
    }
    else if (i%15==0)
    {
      i%30==0 ? rotateLeft() : rotateRight(); 
    }
    if ((i%100==0 || i == 150) && i > 0)
    {
      Serial.print(".");
    }
    qtr.calibrate();
    delay(20);
  }
  Serial.println("");
  Serial.println("Calibration complete");
  
  /*
  qtr.calibrationOn.minimum[0] = 250;
  qtr.calibrationOn.minimum[1] = 257;
  qtr.calibrationOn.minimum[2] = 273;
  qtr.calibrationOn.minimum[3] = 435;
  qtr.calibrationOn.minimum[4] = 263;
  qtr.calibrationOn.minimum[5] = 177;
  qtr.calibrationOn.minimum[6] = 181;
  qtr.calibrationOn.minimum[7] = 252;
  qtr.calibrationOn.maximum[0] = 984;
  qtr.calibrationOn.maximum[1] = 972;
  qtr.calibrationOn.maximum[2] = 974;
  qtr.calibrationOn.maximum[3] = 979;
  qtr.calibrationOn.maximum[4] = 969;
  qtr.calibrationOn.maximum[5] = 959;
  qtr.calibrationOn.maximum[6] = 966;
  qtr.calibrationOn.maximum[7] = 978;
  */

  // Initiate Gripper
  pinMode (gripper, OUTPUT);

  // Start 'Animation'
  delay(150);
  openGripper();
  delay(150);
  closeGripper();
  delay(150);
  openGripper();
  delay(150);
  closeGripper();
}

void loop()
{
  if(speedforce == rotatespeed)
 {
  
 }
 loopcount++; 

  // Start Debug line
  Serial.println("===============================================================");
  Serial.println("");

  // Get the distance to anything in front of it
  distance = getDistance();
  
  // Output Distance
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println("cm");

  // Divide Debug line
  Serial.println("-----------------------------------------------------");
  Serial.println("");

  // Control Gripper based on Distance
  if (distance < 5)
  {
    closeGripper();
  }
  else
  {
    openGripper();
  }

  // Check the sensors and output the values
  uint16_t sensors[8];
  uint16_t sensors_threshold[8];
 readLineblack = qtr.readLineBlack(sensors);

  for (uint16_t i = 0; i < 8; i++)
  {
    sensors_threshold[i] = sensors[i]/lineThreshold;
    Serial.print(sensors_threshold[i]);
    Serial.print("\t");
    if (i == 7) { Serial.println(""); }
  }
 
  // Control Wheels based on Distance
    if(readLineblack > 0)
    {
      if(readLineblack <= 2500)
      {
        speedforce = rotatespeed;
        rotateLeft();
      }
      else if(readLineblack >= 4500)
      {
        speedforce = rotatespeed;
        rotateRight();
      }
      else
      {
        speedforce = drivespeed;
        driveFwd();
      }
      
    }
    else
    {
      speedforce = rotatespeed;
     rotateLeft();
     
    }
 
  // Divide Debug line
  Serial.println("-----------------------------------------------------");
  Serial.println("");
}


// Calculate Distance
int getDistance()
{
  // Echo Locator
  digitalWrite(trig, LOW);
  delayMicroseconds(20);
  digitalWrite(trig, HIGH);
  delayMicroseconds(100);
  digitalWrite(trig, LOW);

  // Receive Distance
  duration = pulseIn(echo, HIGH);
  return duration / 2 * 0.034;
}

// Gripper
void openGripper() {analogWrite(gripper, openedAngle);}
void closeGripper() {analogWrite(gripper, closedAngle);}

// Forward
void driveFwd() {  driveLeftWheel(); driveRightWheel(); }
void driveLeftWheel() {  analogWrite(leftWheelBwd, 0); analogWrite(leftWheelFwd, speedforce); }
void driveRightWheel() {  analogWrite(rightWheelBwd, 0); analogWrite(rightWheelFwd, speedforce);  }

// Backwards
void driveBwd() { reverseLeftWheel(); reverseRightWheel(); }
void reverseLeftWheel() { analogWrite(leftWheelBwd, speedforce); }
void reverseRightWheel() { analogWrite(rightWheelBwd, speedforce); }

// Breaking
void driveBreak() { breakLeftWheel(); breakRightWheel(); }
void breakLeftWheel() { analogWrite(leftWheelFwd, 0); analogWrite(leftWheelBwd, 0); }
void breakRightWheel() { analogWrite(rightWheelFwd, 0); analogWrite(rightWheelBwd, 0); }

// Rotation
void rotateRight() { driveBreak(); driveLeftWheel(); reverseRightWheel(); }
void rotateLeft() { driveBreak(); driveRightWheel(); reverseLeftWheel();  }
