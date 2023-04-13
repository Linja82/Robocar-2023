////////////////////////////Pin Assignments//////////////////////////////////////

#define PIN_Right_LineTracker   A0
#define PIN_Middle_LineTracker  A1
#define PIN_Left_LineTracker    A2

#define PIN_MPU6050_SDA         A4
#define PIN_MPU6050_SCL         A5

#define PIN_Mode_Switch         2

#define PIN_Motor_Standby       3

#define PIN_LED_Data            4

#define PIN_Right_Motor         5
#define PIN_Left_Motor          6
#define PIN_Motor_R_IN1         7
#define PIN_Motor_L_IN1         8

#define PIN_IR_Receiver         9

#define PIN_Servo1              10
#define PIN_Servo2              11

#define PIN_Ultrasonic_Echo     12
#define PIN_Ultrasonic_Trigger  13

///////////////////////////////Variables/////////////////////////////////////////
float avgLeftLine, avgMiddleLine, avgRightLine;
float variance = 10.0;
float leftCurrent, middleCurrent, rightCurrent;
int dark = 50;

#include "motors.h"
#include "line_calibrate.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(PIN_Right_LineTracker, INPUT);
  pinMode(PIN_Middle_LineTracker, INPUT);
  pinMode(PIN_Left_LineTracker, INPUT);

  calibrate();    // Run the calibration function
}

void loop() {

}

