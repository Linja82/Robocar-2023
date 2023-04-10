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

//////////////////////////// Servo Things ///////////////////////////////////////
Servo panServo;
int panAngle = 90;

////////////////////////////// Main Code ////////////////////////////////////////
void setup() {
  // Serial port initialization
  Serial.begin(9600);

  // Servo initialization
  panServo.attach(PIN_Servo1);
  panServo.write(panAngle);
  Serial.println("Pan Servo initalized at " + String(panAngle) + "degrees");

  // MPU6050 Initialization
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  mpu.initialize();

  devStatus = mpu.dmpInitialize();
      // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(54);     // mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(-17);    // mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(47);     // mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();
  mpu.setDMPEnabled(true);
  
  // Drive initialization
  pinMode(PIN_Right_Motor, OUTPUT);
  pinMode(PIN_Left_Motor, OUTPUT);
  pinMode(PIN_Motor_R_IN1, OUTPUT);
  pinMode(PIN_Motor_L_IN1, OUTPUT);
  pinMode(PIN_Motor_Standby, OUTPUT);
  
  // Line Track Initialization
  pinMode(PIN_Left_LineTracker, INPUT);
  pinMode(PIN_Middle_LineTracker, INPUT);
  pinMode(PIN_Right_LineTracker, INPUT);
}

void loop() {
  maze();
  
  exit(0);
}


