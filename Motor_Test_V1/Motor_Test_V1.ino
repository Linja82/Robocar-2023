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

/////////////////////////////Code be here////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_Right_Motor, OUTPUT);
  pinMode(PIN_Left_Motor, OUTPUT);
  pinMode(PIN_Motor_R_IN1, OUTPUT);
  pinMode(PIN_Motor_L_IN1, OUTPUT);
  pinMode(PIN_Motor_Standby, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(PIN_Motor_Standby, HIGH);
  
  digitalWrite(PIN_Motor_R_IN1, LOW);
  analogWrite(PIN_Right_Motor, 50);
  
  digitalWrite(PIN_Motor_L_IN1, HIGH);
  analogWrite(PIN_Left_Motor, 50);
  delay(1000);
  analogWrite(PIN_Right_Motor, 0);
  analogWrite(PIN_Left_Motor, 0);
  delay(5000);
}
