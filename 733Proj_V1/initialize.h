void initialize() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Initialize Pan Servo
  panServo.attach(PIN_Servo1);
  panServo.write(panAngle);
  Serial.println("Pan servo initialized at " + String(panAngle) + " degrees");

  // Line tracker Pin Modes
  pinMode(PIN_Left_LineTracker, INPUT);
  pinMode(PIN_Middle_LineTracker, INPUT);
  pinMode(PIN_Right_LineTracker, INPUT);

  // Motor driver Pin Modes
  pinMode(PIN_Right_Motor, OUTPUT);
  pinMode(PIN_Left_Motor, OUTPUT);
  pinMode(PIN_Motor_R_IN1, OUTPUT);
  pinMode(PIN_Motor_L_IN1, OUTPUT);
  pinMode(PIN_Motor_Standby, OUTPUT);

  // MPU6050 Initialization
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    // Turn on the Digital Motion Processor (DMP), now that it's ready
    
  }
}