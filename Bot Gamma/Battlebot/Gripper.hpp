// Gripper
const int gripper = 10;
const int closedAngle = 180;
const int openedAngle = 130;

void setupGripper()
{
  // Initiate Gripper
  pinMode(gripper, OUTPUT);
}

// Gripper
void openGripper()
{
  analogWrite(gripper, openedAngle);
}

void closeGripper()
{
  analogWrite(gripper, closedAngle);
}