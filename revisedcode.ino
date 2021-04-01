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

// BlueTooth variables used to read in Module, sensors, and remote access
SoftwareSerial bluetoothModule(BLUETOOTH_TX, BLUETOOTH_RX);
bool BluetoothControls();
bool LeftSensor = false;
bool CenterSensor = false;
bool RightSensor = false;
bool BackSensor = false;
bool forwardAllowed = true;
bool reverseAllowed = true;

void setup()
{
  Serial.begin(19200);

  // Joystick read in calls
  pinMode(JOYSTICK_XPIN, INPUT);
  pinMode(JOYSTICK_YPIN, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  // Sensors connected to Arduino Pins
  pinMode(SENSOR0_ECHO, INPUT);
  pinMode(SENSOR0_TRIG, OUTPUT);
  pinMode(SENSOR1_ECHO, INPUT);
  pinMode(SENSOR1_TRIG, OUTPUT);
  pinMode(SENSOR2_ECHO, INPUT);
  pinMode(SENSOR2_TRIG, OUTPUT);
  pinMode(SENSOR3_ECHO, INPUT);
  pinMode(SENSOR3_TRIG, OUTPUT);
  pinMode(SENSOR4_ECHO, INPUT);
  pinMode(SENSOR4_TRIG, OUTPUT);
  pinMode(SENSOR5_ECHO, INPUT);
  pinMode(SENSOR5_TRIG, OUTPUT);

  // Motor read in calls
  lMotorControl.attach(MOTOR_LPIN);
  rMotorControl.attach(MOTOR_RPIN);

  Serial.println("App Started");

  // When it comes to the bluetooth this is going to be the biggest hickup
  // Depending on the module that is used this can be any baud rate
  // This was the baud rate for my Module, but their's might be 9600
  bluetoothModule.begin(115200);
}

void loop()
{
  // If remote access has not been activated
  // If the car is disabled, check for input from bluetooth and loop again
  if(!car_enabled)
  {
      SetMotorIdle();
      BluetoothControls();
      delayMicroseconds(100);

      return;
  }
  // Check sensors,
  // if we are approaching a wall then stop
  LeftSensor = SensorsDetectWall(SENSOR0_TRIG, SENSOR0_ECHO);

  forwardAllowed = true;
  reverseAllowed = true;

  if(LeftSensor)
  {
    forwardAllowed = false;
    SetMotorForwardIdle();
  }

  BluetoothControls();

  if(BL_ForwCount > 0)
  {
      Serial.println("Bluetooth: ForwardSpeed");
      BL_ForwCount--;
      SetMotorForwardSpeed(forwardAllowed);
  }
  else if (BL_RevCount > 0)
  {
      Serial.println("Bluetooth: ReveseSpeed");
      BL_RevCount--;
      SetMotorReveseSpeed(reverseAllowed);
  }
  // Read from joystick if nothing else has stopped us
  else
  {
      //Serial.println("Reading Joysticks");
      //ReadJoystick();
      Serial.println("Reading Button");
      ReadButton();
  }

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

bool SensorsDetectWall(int trigPin, int echoPin)
{
    float duration, sensorDistance;

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    sensorDistance = (duration * .0343)/2;
    Serial.print("Distance: ");
    Serial.println(sensorDistance);
    if (sensorDistance < Distance)
    {
        return true;
    }
    return false;
}

bool BluetoothControls()
{
    if(bluetoothModule.available())
    {
        // Read in the command from bluetooth and process the command
        bluetoothCmd data = (bluetoothCmd)bluetoothModule.read();
        switch(data)
        {
            // For both ForwardSpeed and ReveseSpeed we are pulsing the car
            // This is in case data is not constantly coming in
            case Forward:
                BL_ForwCount = 15;
                BL_RevCount = 0;
                break;
            case Backward:
                BL_RevCount = 15;
                BL_ForwCount = 0;
                break;
            case Left:
                SetMotorTurning(LeftCmd);
                break;
            case Right:
                SetMotorTurning(RightCmd);
                break;
            case Stop:
                car_enabled = false;
                BL_ForwCount = 0;
                BL_RevCount = 0;
                break;
            case Start:
                car_enabled = true;
                BL_ForwCount = 0;
                BL_RevCount = 0;
                break;
            default:
                // Do nothing, garbage data
                break;
        }
        return true;
    }
    return false;
}
