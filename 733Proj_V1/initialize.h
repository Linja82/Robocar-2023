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
}