#include <QTRSensors.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// Define the data transmit/receive pins in Arduino
#define TxD 2
#define RxD 3

SoftwareSerial mySerial(RxD, TxD); // RX, TX for Bluetooth

// Gripper
const int gripper = 13;
const int closedAngle = 40;
const int openedAngle = 100;

// Clicker
const int echo = 12; // Speaker
const int trig = 11; // Microphone
long duration;
int distance;

// Motor
const int leftWheelFwd = 9;
const int leftWheelBwd = 8;
const int rightWheelBwd = 7;
const int rightWheelFwd = 6;

// Sensors
QTRSensors qtr;
const int lineThreshold = 750;

// Servo
Servo servo;

void setup()
{
  mySerial.begin(9600); // For Bluetooth
  
  // Echo locator
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // Wheels
  pinMode(leftWheelFwd, INPUT);
  pinMode(leftWheelBwd, INPUT);
  pinMode(rightWheelFwd, INPUT);
  pinMode(rightWheelBwd, INPUT);

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
    if ((i%100==0 || i == 150) && i > 0)
    {
      Serial.print(".");
    }
    qtr.calibrate();
    delay(20);
  }
  Serial.println("");
  Serial.println("Calibration complete");

  // Initiate Gripper
  servo.attach(gripper);

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
  if (distance < 15)
  {
    closeGripper();
  }
  else
  {
    openGripper();
  }
  
  // Control Wheels based on Distance
  if (distance < 30)
  {
    driveBreak();
  }
  else
  {
    driveFwd();
  }

  // Check the sensors and output the values
  uint16_t sensors[8];
  qtr.readLineBlack(sensors);

  for (uint16_t i = 0; i < 8; i++)
  {
    Serial.print(sensors[i]/lineThreshold);
    Serial.print("\t");
    if (i == 7) { Serial.println(""); }
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
void openGripper() { servo.write(openedAngle); }
void closeGripper() { servo.write(closedAngle); }

// Forward
void driveFwd() { driveLeftWheel(); driveRightWheel(); }
void driveLeftWheel() { breakLeftWheel(); analogWrite(leftWheelFwd, 255); }
void driveRightWheel() { breakRightWheel(); analogWrite(rightWheelFwd, 255); }

// Backwards
void driveBwd() { reverseLeftWheel(); reverseRightWheel(); }
void reverseLeftWheel() { breakLeftWheel(); analogWrite(leftWheelBwd, 255); }
void reverseRightWheel() { breakRightWheel(); analogWrite(rightWheelBwd, 255); }

// Breaking
void driveBreak() { breakLeftWheel(); breakRightWheel(); }
void breakLeftWheel() { analogWrite(leftWheelFwd, 0); analogWrite(leftWheelBwd, 0); }
void breakRightWheel() { analogWrite(rightWheelFwd, 0); analogWrite(rightWheelBwd, 0); }

// Rotation
void rotateRight() { driveLeftWheel(); reverseRightWheel(); }
void rotateLeft() { driveRightWheel(); reverseLeftWheel(); }
