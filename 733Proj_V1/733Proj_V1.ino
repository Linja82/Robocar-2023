#define PIN_Right_LineTracker   A0
#define PIN_Middle_LineTracker  A1
#define PIN_Left_LineTracker    A2

#define PIN_MPU6050_SDA         A4
#define PIN_MPU6050_SCL         A5

#define PIN_LED_Data            4

#define PIN_Right_Motor         5
#define PIN_Left_Motor          6

#define PIN_Servo1              10
#define PIN_Servo2              11

#define PIN_Ultrasonic_Echo     12
#define PIN_Ultrasonic_Trigger  13

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(PIN_Left_LineTracker, INPUT);
  pinMode(PIN_Middle_LineTracker, INPUT);
  pinMode(PIN_Right_LineTracker, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void lineFollow()
{
  /* Line follow pseudo code
   * If (left line tracker see dark)
   *    More speed to left drive motor
   * else if (right line tracker see dark)
   *    More speed to right drive motor
   * else
   *    Equal speed to left and right drive motor
   */
}
