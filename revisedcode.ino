/*
Go Baby Go! Project - Red Team Senior Design 2020-2021
Designed in collaboration with the Computer Engineering / Electrical Engineering Team
This program is able to interface a BlueTooth module for remote control and provide
manual steering and power from a joystick
Highlights of the car's manuverability include zero-radius turning, gradual acceleration
and deceleration, as well as high torque values and slow speed values for safety and functionality
Lead designers: Bryan Serrano, Max Everett
*/


#include <Servo.h>
#include "redteamdefinitions.h"


void setup()
{

  // Joystick read in calls
  pinMode(JOYSTICK_XPIN, INPUT);
  pinMode(JOYSTICK_YPIN, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  // Motor read in calls
  lMotorControl.attach(MOTOR_LPIN);
  rMotorControl.attach(MOTOR_RPIN);

  Serial.println("App Started");

}

void loop()
{

  //read joystick input and save it to a variable
  joyXVal = analogRead(JOYSTICK_XPIN);
  joyYVal = analogRead(JOYSTICK_YPIN);

  if ( JOYSTICK_LEFT <  JOYSTICK_RIGHT)
      (joyXVal <  JOYSTICK_ZERO_X) ? turnVal = giveDirection('L') : sturnVal = giveDirection('R');

  if ( JOYSTICK_LEFT >  JOYSTICK_RIGHT)
      (joyXVal >  JOYSTICK_ZERO_X) ? turnVal = giveDirection('L') : turnVal = giveDirection('R');

  if ( JOYSTICK_UP <  JOYSTICK_DOWN)
    speedVal = (joyYVal < JOYSTICK_ZERO_Y) ? speedVal = giveDirection('U') : speedVal = giveDirection('D');

  if ( JOYSTICK_UP >  JOYSTICK_DOWN)
    speedVal = (joyYVal > JOYSTICK_ZERO_Y) ? speedVal = giveDirection('U') : speedVal = giveDirection('D');

  lMotorThold = speedVal + turnVal;
  rMotorThold = speedVal - turnVal;


  // left motor movement block that sets proper turning behavior
  if (lMotorThold < MOTOR_MAXTHOLD && lMotorThold > MOTOR_MINTHOLD)
    lMotorThold = THRESHOLD;

  if (lMotorThold < THRESHOLD)
  {
    if (lMotorThold <= lMotorSpeed - ACC_BACKWARD)
      lMotorSpeed = lMotorSpeed - ACC_BACKWARD;

    if (lMotorThold >lMotorSpeed + DEC_BACKWARD)
      lMotorSpeed =lMotorSpeed + DEC_BACKWARD;
  }

  if (lMotorThold >= THRESHOLD)
  {
    if (lMotorThold >= lMotorSpeed + ACC_FORWARD)
      lMotorSpeed = lMotorSpeed + ACC_FORWARD;

    if (lMotorThold < lMotorSpeed - DEC_FORWARD)
      lMotorSpeed = lMotorSpeed - DEC_FORWARD;
  }

  // right motor movement block that sets proper turning behavior
  if (rMotorThold < MOTOR_MAXTHOLD && rMotorThold > MOTOR_MINTHOLD)
    rMotorThold = THRESHOLD;
  if (rMotorThold < THRESHOLD)
  {
    if (rMotorThold <= rMotorSpeed - ACC_BACKWARD)
      rMotorSpeed = rMotorSpeed - ACC_BACKWARD;
    if (rMotorThold > rMotorSpeed + DEC_BACKWARD)
      rMotorSpeed = rMotorSpeed + DEC_BACKWARD;
  }
  if (rMotorThold >= THRESHOLD)
  {
    if (rMotorThold >= rMotorSpeed + ACC_FORWARD)
      rMotorSpeed = rMotorSpeed + ACC_FORWARD;
    if (rMotorThold < rMotorSpeed - DEC_FORWARD)
      rMotorSpeed = rMotorSpeed - DEC_FORWARD;
  }

  lMotorSpeed = constrain(lMotorSpeed, MAX_BACKWARD, MAX_FORWARD);
  rMotorSpeed = constrain(rMotorSpeed, MAX_BACKWARD, MAX_FORWARD);
  rMotorControl.writeMicroseconds(int(rMotorSpeed));
  lMotorControl.writeMicroseconds(int(lMotorSpeed));

}

float giveDirection(char direction)
{
  switch(direction)
  {
    case 'L' :
      return map(joyXVal,  JOYSTICK_LEFT,  JOYSTICK_ZERO_X, TURN_SPEED * -500, 0);
      break;
    case 'R' :
      return map(joyXVal,  JOYSTICK_RIGHT,  JOYSTICK_ZERO_X, TURN_SPEED * 500, 0);
      break;
    case 'U' :
      return map(joyYVal,  JOYSTICK_UP,  JOYSTICK_ZERO_Y, MAX_FORWARD, THRESHOLD);
      break;
    case 'D' :
      return map(joyYVal,  JOYSTICK_DOWN,  JOYSTICK_ZERO_Y, MAX_BACKWARD, THRESHOLD);
      break;
    default :
      break;
  }
}
