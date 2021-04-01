// telling the arduino where everything is
#define JOYSTICK_XPIN 16
#define JOYSTICK_YPIN 15
#define MOTOR_LPIN 10
#define MOTOR_RPIN 9
#define MOTOR_MINTHOLD 1484
#define MOTOR_MAXTHOLD 1516

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
#define JOYSTICK_ZERO 511
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
