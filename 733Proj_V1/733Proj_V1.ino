//////////////////////////////// Notes //////////////////////////////////////////

// Pan Servo has a range of 0 - 180 deg

/* IR Line Trackers
 *
 * Black objects give a reading of 1000+
 * White objects give a reading of ~850 - ~950
 */

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

/////////////////////////////// Imports /////////////////////////////////////////

#include <Servo.h>
#include <FastLED.h>
#include "motors.h"

////////////////////////// Servo Definitions ////////////////////////////////////

Servo panServo;
int panAngle = 90;    // Default servo startup angle. Points straight forward.

////////////////////////////// Variables ////////////////////////////////////////

int motorDifferential = 10;

//////////////////////////// Code be here ///////////////////////////////////////

#include "initialize.h"

int main(void)
{
  initialize();
  lineFollow();
  maze();
}

void lineFollow(){
  /* Line follow pseudo code
   * If (All line trackers see dark)
   *    Intersection detected
   *    Stop at intersection
   *    delay(3500);
   * else if (left line tracker see dark)
   *    More speed to left drive motor
   * else if (right line tracker see dark)
   *    More speed to right drive motor
   * else
   *    Equal speed to left and right drive motor
   */
}

void maze(){
  /* Maze traverse pseudo code
   * Move forward 1 unit
   * Scan left
   * Scan middle
   * Scan Right
   * Rotate to whichever direction has the largest measured distance
   */
}
