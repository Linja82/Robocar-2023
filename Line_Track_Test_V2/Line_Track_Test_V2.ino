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
float variance = 5.0;
float leftCurrent, middleCurrent, rightCurrent;

#include "motors.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(PIN_Right_LineTracker, INPUT);
  pinMode(PIN_Middle_LineTracker, INPUT);
  pinMode(PIN_Left_LineTracker, INPUT);
}

void loop() {
  leftCurrent = digitalRead(PIN_Left_LineTracker);
  rightCurrent = digitalRead(PIN_Right_LineTracker);
  middleCurrent = digitalRead(PIN_Middle_LineTracker);

  Serial.println("Left: " + String(leftCurrent) + " Middle: " + String(middleCurrent) + " Right: " + String(rightCurrent));

  if (leftCurrent > 0) {
    Serial.println("Left line tracker detected tape. Turning left.");
    drive("DIFFERENTIAL LEFT", 35, 0);
  }
  else if (rightCurrent > 0) {
    Serial.println("Right line tracker detected tape. Turning right.");
    drive("DIFFERENTIAL RIGHT", 0, 35);
  }
  else {
    Serial.println("No tape detected by left or right tracker. Driving straight.");
    drive("FORWARD", 35, 35);
  }
}

