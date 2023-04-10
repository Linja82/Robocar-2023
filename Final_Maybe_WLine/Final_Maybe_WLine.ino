//////////////////////////////// Notes //////////////////////////////////////////

// Pan Servo has a range of 0 - 180 deg

// Yaw: Right is +, Left is -
// Pitch: Up is +, Down is -
// Roll: Left is +, Right is -

// Drive direction: HIGH for IN1 indicates forward wheel rotation, LOW indicates reverse wheel rotation

/////////////////////////// Pin Assignments /////////////////////////////////////

#define PIN_Right_LineTracker A0
#define PIN_Middle_LineTracker A1
#define PIN_Left_LineTracker A2

#define PIN_VBat A3

#define PIN_MPU6050_SDA A4
#define PIN_MPU6050_SCL A5

#define PIN_Mode_Switch 2

#define PIN_Motor_Standby 3

#define PIN_LED_Data 4

#define PIN_Right_Motor 5
#define PIN_Left_Motor 6
#define PIN_Motor_R_IN1 7
#define PIN_Motor_L_IN1 8

#define PIN_IR_Receiver 9

#define PIN_Servo1 10
#define PIN_Servo2 11

#define PIN_Ultrasonic_Echo 12
#define PIN_Ultrasonic_Trigger 13

/////////////////////////////// Imports /////////////////////////////////////////

#include <Servo.h>
#include <NewPing.h>                     // https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
#include "I2Cdev.h"                      // https://github.com/jrowberg/i2cdevlib
#include "MPU6050_6Axis_MotionApps20.h"  // https://github.com/jrowberg/i2cdevlib
#include "Maze.h"
#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"
// #include "lineCount.h"

Servo panServo;
int panAngle = 90;

int blackLines = 0;

////////////////////////////// Main Code ////////////////////////////////////////
void setup() {
  // Serial port initialization
  Serial.begin(9600);

  // Servo initialization
  // panServo.attach(PIN_Servo1);
  // panServo.write(panAngle);
  // Serial.println("Pan Servo initalized at " + String(panAngle) + "degrees");
  
  // Line Track Initialization
  // pinMode(PIN_Left_LineTracker, INPUT);
  // pinMode(PIN_Middle_LineTracker, INPUT);
  // pinMode(PIN_Right_LineTracker, INPUT);
  
  Application_FunctionSet.ApplicationFunctionSet_Init();
}

void loop() {
  lineTrack();
 
  delay(3000);
  // maze();
  
  exit(0);
}

int S = 250;
int E = 850;

void lineTrack(){
  while (blackLines < 2) {
    delay(10);
    Application_FunctionSet.ApplicationFunctionSet_Tracking(); 
    Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
    
    if (line_comparator(analogRead(PIN_Middle_LineTracker), S, E) && line_comparator(analogRead(PIN_Left_LineTracker), S, E) && line_comparator(analogRead(PIN_Right_LineTracker), S, E)) {
      blackLines += 1;
    }
  } 
  
  digitalWrite(PIN_Motor_Standby, LOW);    // Disables the motor driver
}

bool line_comparator(long x, long s, long e) //f(x)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}


