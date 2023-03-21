// Gripper
const int gripper = 10;
const int closedAngle = 130;
const int openedAngle = 180;

void setupGripper()
{
  // Initiate Gripper
  pinMode(gripper, OUTPUT);
}

// Gripper
void openGripper()
{
  neoFull(0,150,0);
  analogWrite(gripper, openedAngle);
}

void closeGripper()
{
  neoFull(0,0,150);
  analogWrite(gripper, closedAngle);
}