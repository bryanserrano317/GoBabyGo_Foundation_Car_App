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
#include <SoftwareSerial.h>

// telling the arduino where everything is
#define JOYSTICK_XPIN 16
#define JOYSTICK_YPIN 15
#define MOTOR_LPIN 10
#define MOTOR_RPIN 9
#define MOTOR_MINTHOLD 1484
#define MOTOR_MAXTHOLD 1516

#define SENSOR0_ECHO        30
#define SENSOR0_TRIG        31
#define SENSOR1_ECHO        32
#define SENSOR1_TRIG        33
#define SENSOR2_ECHO        34
#define SENSOR2_TRIG        35
#define SENSOR3_ECHO        36
#define SENSOR3_TRIG        37
#define SENSOR4_ECHO        38
#define SENSOR4_TRIG        39
#define SENSOR5_ECHO        40
#define SENSOR5_TRIG        41

// power block. higher level than sensitivity
Servo rMotorControl; // rightMotorController
Servo lMotorControl; // Servo leftMotorController;
float lMotorSpeed = 1500.00;
float rMotorSpeed = 1500.00;
float rMotorThold = 1500.00;
float lMotorThold = 1500.00;
float joyXVal = 0000.000;
float joyYVal = 0000.000;
unsigned int timezie = 0;
float speedVal = 1500.000;
float turnVal = 0000.000;

// Joystick sensitivity block
#define JOYSTICK_ZERO_X 509
#define JOYSTICK_RIGHT 632
#define JOYSTICK_LEFT 385
#define JOYSTICK_UP 638
#define JOYSTICK_ZERO_Y 511
#define JOYSTICK_DOWN 380

// motor sensitivity block (i.e. acceleration)
#define ACC_FORWARD   .2
#define DEC_FORWARD   .6
#define ACC_BACKWARD  .1
#define DEC_BACKWARD  .8
#define MAX_FORWARD 1750
#define MAX_BACKWARD 1250
#define TURN_SPEED .6
#define THRESHOLD 1500.00


SoftwareSerial bluetoothModule(BLUETOOTH_TX, BLUETOOTH_RX);

// Should the car be enabled or disabled based on Bluetooth
bool car_enabled = true;

// Speed values
signed int ReveseSpeed = 0;
signed int ForwardSpeed = 0;

// Bluetooth inputs will come suddenly and infrequently, and we don't want to suddently jolt the car
// Instead of sending data constantly we will get a single command and repeat it so many times
int BL_ForwCount = 0;
int BL_RevCount = 0;

//Define functions
bool SensorsDetectWall(int trigPin, int echoPin);
bool BluetoothControls();
/*
void ReadJoystick();
void SetMotorForwardSpeed(bool forward);
void SetMotorReveseSpeed(bool reverse);
void SetMotorIdle();
*/
bool SetMotorTurning();

//boolean variables for distance sensors
bool LeftSensor = false;
bool CenterSensor = false;
bool RightSensor = false;
bool BackSensor = false;

//boolean variables for moving forward
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

  pinMode(BUTTON, OUTPUT);

  digitalWrite(BUTTON, HIGH);


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

  // right motor movement bloc#include <Servo.h>

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

/*void SetMotorForwardIdle()
{
   // Serial.println("Motor idling");
   ForwardSpeed = lim_max(0, (ForwardSpeed-Deccleration));
    //ReveseSpeed = lim_max(0, (ReveseSpeed-Deccleration));

    analogWrite(MOTOR1_PWM_CW, ForwardSpeed);
    analogWrite(MOTOR2_PWM_CCW, ForwardSpeed);
  //  analogWrite(MOTOR2_PWM_CW, ReveseSpeed);
  //  analogWrite(MOTOR1_PWM_CCW, ReveseSpeed);
}

void ReadButton()
{
    if(digitalRead(BUTTON) == LOW)  // If button pressed
    {
      SetMotorForwardSpeed(forwardAllowed);
    }
    else
    {
        SetMotorIdle();
    }
}


void SetMotorForwardSpeed(bool forward)
{
  if (!forward)
  {
    Serial.println("Can't move forward");
    return;
  }
  else
  {
    Serial.println("Forwarding");
    ForwardSpeed = lim_min(MaxSpeed, ForwardSpeed+Accleration);
    ReveseSpeed = lim_max(0, ReveseSpeed-Deccleration);

    analogWrite(MOTOR1_PWM_CW, ForwardSpeed);
    analogWrite(MOTOR1_PWM_CCW, ReveseSpeed);
    analogWrite(MOTOR2_PWM_CW, ReveseSpeed);
    analogWrite(MOTOR2_PWM_CCW, ForwardSpeed);
  }
}

// Increase the ReveseSpeed speed of both of the motors and decrease their ForwardSpeed speed
void SetMotorReveseSpeed(bool reverse)
{
  if (!reverse)
  {
    Serial.println("Can't move reverse");
    return;
  }
  else{

    Serial.println("Reversing");
    ReveseSpeed = lim_min(MaxSpeed, ReveseSpeed+Accleration);
    ForwardSpeed = lim_max(0, ForwardSpeed-Deccleration);

    analogWrite(MOTOR1_PWM_CW, ForwardSpeed);
    analogWrite(MOTOR1_PWM_CCW, ReveseSpeed);
    analogWrite(MOTOR2_PWM_CW, ReveseSpeed);
    analogWrite(MOTOR2_PWM_CCW, ForwardSpeed);
  }
}

// Slowly break the motors, this prevents sudden breaking
void SetMotorIdle()
{
    Serial.println("Motor idling");
    ForwardSpeed = lim_max(0, (ForwardSpeed-Deccleration));
    ReveseSpeed = lim_max(0, (ReveseSpeed-Deccleration));

    analogWrite(MOTOR1_PWM_CW, ForwardSpeed);
    analogWrite(MOTOR2_PWM_CCW, ForwardSpeed);
    analogWrite(MOTOR2_PWM_CW, ReveseSpeed);
    analogWrite(MOTOR1_PWM_CCW, ReveseSpeed);
}

// Slowly break the motors, this prevents sudden breaking
void SetMotorForwardIdle()
{
   // Serial.println("Motor idling");
   ForwardSpeed = lim_max(0, (ForwardSpeed-Deccleration));
    //ReveseSpeed = lim_max(0, (ReveseSpeed-Deccleration));

    analogWrite(MOTOR1_PWM_CW, ForwardSpeed);
    analogWrite(MOTOR2_PWM_CCW, ForwardSpeed);
  //  analogWrite(MOTOR2_PWM_CW, ReveseSpeed);
  //  analogWrite(MOTOR1_PWM_CCW, ReveseSpeed);
}*/

// Turn steering motor either right or left
bool SetMotorTurning(turnCmd turn)
{
    switch(turn)
    {
      LeftCmd:
          // Rotate the stepper motor once Left
          return true;
      RightCmd:
          // Rotate the stepper motor once Right
          return true;
      default:
          // Garbage data, do nothing
          return false;
    }
}
