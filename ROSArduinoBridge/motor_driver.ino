/***************************************************************
   Motor driver definitions

   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   *************************************************************/

#ifdef USE_BASE
#include <Servo.h>
Servo esc_lf; // left front;
Servo esc_rf; // right front;
Servo esc_lr; // left rear;
Servo esc_rf; // right rear;

#ifdef defined L298_MOTOR_DRIVER
void initMotorController()
{
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
}

void setMotorSpeed(int i, int spd)
{
  unsigned char reverse = 0;

  if (spd < 0)
  {
    spd = -spd;
    reverse = 1;
  }
  if (spd > 255)
    spd = 255;

  if (i == LEFT)
  {
    if (reverse == 0)
    {
      analogWrite(LEFT_MOTOR_FORWARD, spd);
      analogWrite(LEFT_MOTOR_BACKWARD, 0);
    }
    else if (reverse == 1)
    {
      analogWrite(LEFT_MOTOR_BACKWARD, spd);
      analogWrite(LEFT_MOTOR_FORWARD, 0);
    }
  }
  else /*if (i == RIGHT) //no need for condition*/
  {
    if (reverse == 0)
    {
      analogWrite(RIGHT_MOTOR_FORWARD, spd);
      analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    }
    else if (reverse == 1)
    {
      analogWrite(RIGHT_MOTOR_BACKWARD, spd);
      analogWrite(RIGHT_MOTOR_FORWARD, 0);
    }
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}

#elif defined YELLOW_JACKET_DRIVER

void initMotorController()
{
}

void setMotorSpeed(int i, int spd)
{

  const int SIGNAL_MIN =1050;
  const int SIGNAL_MAX = 1950;
  const int SPEED_MIN=-100;
   const int SPEED_MAX= 100;
  // set the speed range;
  spd = constrain(spd,SPEED_MIN,SPEED_MAX);
  int signal_output = map(spd,SPEED_MIN,SPEED_MAX,SIGNAL_MIN,SIGNAL_MAX);

  if (i == LEFT_FRONT)
  { 
    esc_lf.writeMicroseconds(signal_output);
  }
  else if (i == RIGHT_FRONT)
  {
    esc_rf.writeMicroseconds(signal_output);
  }
  else if (i == LEFT_REAR)
  {
    esc_lr.writeMicroseconds(signal_output);
  }
  else
  {
    esc_rr.writeMicroseconds(signal_output);
  }
}

void setMotorSpeeds(int leftFrontSpeed, int rightFrontSpeed, int leftRearSpeed, int rightRearSpeed)
{
  setMotorSpeed(LEFT_FRONT, leftFrontSpeed);
  setMotorSpeed(RIGHT_FRONT, rightFrontSpeed);
  setMotorSpeed(LEFT_REAR, leftRearSpeed);
  setMotorSpeed(RIGHT_REAR, rightRearSpeed);
}

#else
#error A motor driver must be selected!
#endif

#endif
