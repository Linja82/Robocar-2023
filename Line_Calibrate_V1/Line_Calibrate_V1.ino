/////////////////////////// Pin Assignments /////////////////////////////////////

#define PIN_Right_LineTracker   A0
#define PIN_Middle_LineTracker  A1
#define PIN_Left_LineTracker    A2

#define PIN_VBat                A3

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

/////////////////////// Line Follow Variables ///////////////////////////////////
float avgLeftLine, avgMiddleLine, avgRightLine;

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_Right_LineTracker, INPUT);
  pinMode(PIN_Middle_LineTracker, INPUT);
  pinMode(PIN_Left_LineTracker, INPUT);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}

void calibrate() {
  float leftReadings[10];
  float middleReadings[10];
  float rightReadings[10];
  
  for (int i = 0; i < 10; i++) {
    leftReadings[i] = analogRead(PIN_Left_LineTracker);
    middleReadings[i] = analogRead(PIN_Middle_LineTracker);
    rightReadings[i] = analogRead(PIN_Right_LineTracker);
    
    delay(200);
  }
  
  float leftSum, middleSum, rightSum;
  
  for (int i = 0; i < 10; i++) {
    leftSum += leftReadings[i];
    middleSum += middleReadings[i];
    rightSum += rightReadings[i];
  }
  
  avgLeftLine = leftSum / 10.0;
  avgMiddleLine = middleSum / 10.0;
  avgRightLine = rightSum / 10.0;
}