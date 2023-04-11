/////////////////////// Function Initializations ////////////////////////////////
float getGyro(String angle);
void drive_straight();
void turn_Left();
void turn_Right();
void measure_distance();
void stop();

///////////////////////////// Drive Things //////////////////////////////////////
#define speed 60        // Forward driving speed
#define turnSpeed 45    // Speed for variable turning, left and right motor have inverted speeds
#define speedShift 20   // Speed difference between left/right when driving straight with a slight differential
#define variance 3      // Number of degrees up or down that is allowed away from the target

////////////////////////// Ultrasonic Things ////////////////////////////////////
#define MAX_DISTANCE 300  // Maximum distance the ultrasonic will try to measure in cm
#define ITERATIONS 5      // Number of iterations to use for median measurement

NewPing sonar(PIN_Ultrasonic_Trigger, PIN_Ultrasonic_Echo, MAX_DISTANCE);

int distance;
#define approachDist  9

/////////////////////////// MPU6050 Things //////////////////////////////////////
MPU6050 mpu;    // MPU object

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q1;          // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Angle variables
float yaw, roll, pitch;
float yawTarget, pitchTarget;
#define rampAngle 3     // Angle at which the ramp is detected

void mpuForMaze() {   // Function to initialize all the devices needed for the maze
  // Drive initialization
  pinMode(PIN_Right_Motor, OUTPUT);
  pinMode(PIN_Left_Motor, OUTPUT);
  pinMode(PIN_Motor_R_IN1, OUTPUT);
  pinMode(PIN_Motor_L_IN1, OUTPUT);
  pinMode(PIN_Motor_Standby, OUTPUT);
  
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
  mpu.setXGyroOffset(51);     // mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(-17);    // mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(47);     // mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();
  mpu.setDMPEnabled(true);
}

void maze(){
  mpuForMaze();   // Initialize devices required for maze
  // Stage 1, approach ramp
  Serial.println("Stage 1: Approach Ramp");
  
  pitch = getGyro("Pitch");         // Get the current pitch of the car
  pitchTarget = pitch + rampAngle;  // Set the target pitch to the current pitch + the ramp threshold angle
  yawTarget = getGyro("Yaw");       // Set the heading target to the current heading
  
  while (pitch < pitchTarget){
    drive_straight();
    pitch = getGyro("Pitch");
  }
   
  Serial.println("Stage 2: Ramp detected, climbing ramp");
  
  // Stage 2, climb ramp
  pitch = getGyro("Pitch");
  pitchTarget = pitch - rampAngle;
  
  while (pitch > pitchTarget){
    drive_straight();
    pitch = getGyro("Pitch");
  }
  
  Serial.println("Stage 3: Off ramp, approaching Wall 1");

  // Stage 3, Approach first wall
  measure_distance();
  
  while (distance > approachDist){
    drive_straight();
    measure_distance();
  }
  
  stop();
  Serial.println("Stage 4: Turning left to face Wall 2");
  
  // Stage 4, Turn left to face Wall 2
  turn_Left();
  
  stop();
  Serial.println("Stage 5: Approaching Wall 2");
  
  // Stage 5, Drive straight until Wall 2
  measure_distance();
  
  while (distance > approachDist){
    drive_straight();
    measure_distance();
  }
  
  stop();
  Serial.println("Stage 6: Turning left to face Wall 3");
  
  // Stage 6, Turn Left to face Wall 3
  turn_Left();
  
  stop();
  Serial.println("Stage 7: Approaching Wall 3");
  
  // Stage 7, Approach Wall 3
  measure_distance();
  
  while (distance > approachDist){
    drive_straight();
    measure_distance();
  }
  
  stop();
  Serial.println("Stage 8: Turn right to face Wall 4");
  
  // Stage 8, Turn right to face Wall 4
  turn_Right();
  
  stop();
  Serial.println("Stage 9: Approach Wall 4");
  
  // Stage 9: Approach Wall 4
  measure_distance();
  
  while (distance > approachDist){
    drive_straight();
    measure_distance();
  }
  
  stop();
  Serial.println("Stage 10: Turn right to face Wall 5");
  
  // Stage 10, Turn right to face Wall 5
  turn_Right();
  
  stop();
  Serial.println("Stage 11: Approach Wall 5");
  
  // Stage 11: Approach Wall 5
  measure_distance();
  
  while (distance > approachDist){
    drive_straight();
    measure_distance();
  }
  
  stop();
  Serial.println("Stage 12: Turn left to face Wall 6");
  
  // Stage 12, Turn left to face Wall 6
  turn_Left();
  
  stop();
  Serial.println("Stage 13: Approach Wall 6");
  
  // Stage 13, Approach Wall 6
  measure_distance();
  
  while (distance > approachDist){
    drive_straight();
    measure_distance();
  }
  
  stop();
  Serial.println("Stage 14: Turn left to face exit");
  
  // Stage 14, Turn left to face exit
  turn_Left();
  
  stop();
  Serial.println("Stage 15: Approach exit");
  
  // Stage 15, Approach exit
  // Basically stop when the front wheels fall off
  pitch = getGyro("Pitch");
  pitchTarget = pitch - rampAngle;
  
  while (pitch > pitchTarget){
    drive_straight();
    pitch = getGyro("Pitch");
  }
  
  stop();
  Serial.println("Maze complete");
}

void measure_distance() {  // Uses the ultrasonic sensor to measure the distance to the wall ahead
  distance = sonar.ping_median(ITERATIONS);
  distance = sonar.convert_cm(distance);
  delay(50);
}

float getGyro(String angle){
  mpu.dmpGetCurrentFIFOPacket(fifoBuffer);  // Get the Latest packet 
  
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q1, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q1);
  mpu.dmpGetYawPitchRoll(ypr, &q1, &gravity);
  
  // ypr is not actually the order for the orientation of MPU6050 that we are using
  // It's Yaw, Roll, then Pitch for our orientation
  
  if (angle.equals("Yaw")){
    return ypr[0] * 180/M_PI;
  }
  else if (angle.equals("Roll")){
    return ypr[1] * 180/M_PI;
  }
  else if (angle.equals("Pitch")){
    return ypr[2] * 180/M_PI;
  }
}

void drive_straight(){
  yaw = getGyro("Yaw");
  float yaw_max = yawTarget + variance;
  float yaw_min = yawTarget - variance;
  
  if (yaw > yaw_max){   // turn slight left
    // Serial.println("Turning slight left");
    digitalWrite(PIN_Motor_Standby, HIGH);    // Enables the motor driver
    
    digitalWrite(PIN_Motor_L_IN1, HIGH);      // Set Left side forward
    digitalWrite(PIN_Motor_R_IN1, HIGH);      // Set Right side forward
    
    analogWrite(PIN_Left_Motor, (speed - speedShift));       // Runs the left motors at [speed] which is a PWM value from 0-255
    analogWrite(PIN_Right_Motor, speed);      // Runs the right motors at [speed] which is a PWM value from 0-255
  }
  else if (yaw < yaw_min){    // turn slight right
    // Serial.println("Turning slight right");
    digitalWrite(PIN_Motor_Standby, HIGH);    // Enables the motor driver
    
    digitalWrite(PIN_Motor_L_IN1, HIGH);      // Set Left side forward
    digitalWrite(PIN_Motor_R_IN1, HIGH);      // Set Right side forward
    
    analogWrite(PIN_Left_Motor, speed);       // Runs the left motors at [speed] which is a PWM value from 0-255
    analogWrite(PIN_Right_Motor, (speed - speedShift));      // Runs the right motors at [speed] which is a PWM value from 0-255
  }
  else{   // drive straight
    // Serial.println("Driving straight");
    digitalWrite(PIN_Motor_Standby, HIGH);    // Enables the motor driver
    
    digitalWrite(PIN_Motor_L_IN1, HIGH);      // Set Left side forward
    digitalWrite(PIN_Motor_R_IN1, HIGH);      // Set Right side forward
    
    analogWrite(PIN_Left_Motor, speed);       // Runs the left motors at [speed] which is a PWM value from 0-255
    analogWrite(PIN_Right_Motor, speed);      // Runs the right motors at [speed] which is a PWM value from 0-255
  }
}

void turn_Left(){
  digitalWrite(PIN_Motor_Standby, HIGH);    // Enables the motor driver
  
  digitalWrite(PIN_Motor_L_IN1, LOW);       // Set Left side reverse
  digitalWrite(PIN_Motor_R_IN1, HIGH);      // Set Right side forward
  
  analogWrite(PIN_Left_Motor, turnSpeed);       // Runs the left motors at [speed] which is a PWM value from 0-255
  analogWrite(PIN_Right_Motor, turnSpeed);      // Runs the right motors at [speed] which is a PWM value from 0-255

  delay(1800);
}

void turn_Right(){
  digitalWrite(PIN_Motor_Standby, HIGH);    // Enables the motor driver

  digitalWrite(PIN_Motor_L_IN1, HIGH);      // Set Left side forward
  digitalWrite(PIN_Motor_R_IN1, LOW);       // Set Right side reverse

  analogWrite(PIN_Left_Motor, turnSpeed);       // Runs the left motors at [speed] which is a PWM value from 0-255
  analogWrite(PIN_Right_Motor, turnSpeed);      // Runs the right motors at [speed] which is a PWM value from 0-255

  delay(1800);
}

void stop(){
  digitalWrite(PIN_Motor_Standby, LOW);    // Disables the motor driver
}