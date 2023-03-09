// Clicker
const int echo = 12; // Speaker
const int trig = 13; // Microphone
long duration;
int distance;

void setupEcho()
{
  // Echo locator
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
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