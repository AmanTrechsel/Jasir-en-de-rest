// Libraries
#include <SoftwareSerial.h>
#include <QTRSensors.h>

// Motor
const int leftWheelFwd = 11; // Links, vooruit
const int leftWheelBwd = 10; // Links, achteruit
const int rightWheelFwd = 9; // Rechts, achteruit
const int rightWheelBwd = 6; // Rechts, vooruit
const int rotationSpeed = 110; // Speed at which to rotate
const int driveSpeed = 210; // Speed at which to drive
const int trigPin = 7; // de pin dat is verbonden met de ultra sonic sensor
const int echoPin = 8; // the pin dat is verbonden met de echo van de ultra sonic sensor
int actualSpeed = 255; // de actuele snelheid van de motor
int duration; // tijd hoelang het duurt
int distance; // afstand 

// Sensors
QTRSensors qtr;
uint16_t sensors[8];
int lineReadData; // Value depending on where the sensor is detecting a line
                  // A value closer to 0 is more left; closer to 7000 is more right; 3500 is center
                  // 0 means no line is detected; 7000 means everything is detecting.

// Sensor Calibration
const int calibrationTime = 75; // in milliseconds * 20 (50 = 1 second)
const bool shouldCalibrate = true;

// Bluetooth configuration
SoftwareSerial configureBT(3, 2); // TX || RX, in feite zijn dit de pinmodes

void setup() {
  // Bluetooth setup 
  Serial.begin(38400);
  configureBT.begin(9600); // Indien je commands wilt uitvoeren, knop op module ingedrukt houden bij het inpluggen en 9600 aanpassen naar 38400

  // Echo sensor 
  pinMode(trigPin, OUTPUT); // zet de trigger pin op een output
  pinMode(echoPin, INPUT); // zet de echo pin op input

    // Wheels
  pinMode(leftWheelFwd, OUTPUT);
  pinMode(leftWheelBwd, OUTPUT);
  pinMode(rightWheelFwd, OUTPUT);
  pinMode(rightWheelBwd, OUTPUT);

  // Line Sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7},8);
  
  // Initiate Serial
  Serial.begin(9600);

  // Sensor Calibration
  if (shouldCalibrate)
  {
    int i;
    Serial.println("");
    Serial.print("Calibrating");
    for (i = 0; i < calibrationTime; i++)
    {
      driveBreak();
      driveFwd();
      delay(10);
      slowDriveFwd();
      qtr.calibrate();
      delay(10);
    }
    Serial.println("");
    Serial.println("Calibration complete");
  }
}

void loop() {
  echoSensor(); 
  if (configureBT.available()) {
    Serial.write(configureBT.read());
  }
  
  if (Serial.available()) {
    configureBT.write(Serial.read());
  }

    // Start Debug line
  Serial.println("===============================================================");
  Serial.println("");

  // Divide Debug line
  Serial.println("-----------------------------------------------------");
  Serial.println("");

  // Check the sensors and output the values
  lineReadData = qtr.readLineBlack(sensors);
  for (uint16_t i = 0; i < 8; i++)
  {
    Serial.print(sensors[i]);
    Serial.print("\t");
    if (i == 7) { Serial.println(lineReadData); }
  }
  if (distance <=16)
  {
    actualSpeed = rotationSpeed;
    rotateRight();
    delay(1000);
    driveFwd();
    delay(1400);
    rotateLeft();
    delay(1000);
    driveFwd();
    delay(1400);
    
    
  }
  else{
  
  // Control Wheels based on Distance
  if (lineReadData > 0)
  {
    if (lineReadData <= 2380)
    {
      actualSpeed = rotationSpeed;
      rotateLeft();
    }
    else if (lineReadData >= 4250)
    {
      actualSpeed = rotationSpeed;
      rotateRight();
    }
    else
    {
      actualSpeed = driveSpeed;
      driveFwd();
    }
  }
  else // Cannot detect any lines
  {
    // Prefer left over right, so rotate right.
    actualSpeed = rotationSpeed;
    rotateLeft();
  }
  }
 
  // Divide Debug line
  Serial.println("-----------------------------------------------------");
  Serial.println("");
  
}

// Forward
void driveFwd()
{
  driveLeftWheel();
  driveRightWheel();
}

void driveLeftWheel()
{
  analogWrite(leftWheelBwd, 0);
  analogWrite(leftWheelFwd, actualSpeed);
}

void driveRightWheel()
{
  analogWrite(rightWheelBwd, 0);
  analogWrite(rightWheelFwd, actualSpeed);
}

void slowDriveFwd()
{
  rightSlowWheelFwd();
  leftSlowWheelFwd();
}

void leftSlowWheelFwd()
{
  analogWrite(rightWheelBwd, 0);
  analogWrite(leftWheelFwd, 50);
}

void rightSlowWheelFwd()
{
  analogWrite(leftWheelBwd, 0);
  analogWrite(rightWheelFwd, 40);
}

// Backwards
void driveBwd()
{
  reverseLeftWheel();
  reverseRightWheel();
}

void reverseLeftWheel()
{
  analogWrite(leftWheelBwd, actualSpeed);
}

void reverseRightWheel()
{
  analogWrite(rightWheelBwd, actualSpeed);
}

// Breaking
void driveBreak()
{
  breakLeftWheel();
  breakRightWheel();
}

void breakLeftWheel()
{
  analogWrite(leftWheelFwd, 50);
  analogWrite(leftWheelBwd, 50);
}

void breakRightWheel()
{
  analogWrite(rightWheelFwd, 50);
  analogWrite(rightWheelBwd, 50);
}

// Rotation
void rotateRight()
{
  driveBreak();
  driveLeftWheel();
  reverseRightWheel();
}

void rotateLeft()
{
  driveBreak();
  driveRightWheel();
  reverseLeftWheel();
}

// echo sensor
void echoSensor() 
{
  digitalWrite(trigPin, LOW);
  
  digitalWrite(trigPin, HIGH);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = duration * 0.034 / 2;

  Serial.print("distance: ");
  Serial.println(distance);
}
