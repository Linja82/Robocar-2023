void drive(String direction, int speed, int distance){
  if (direction.equals("Forward")){
    digitalWrite(PIN_Motor_Standby, HIGH);    // Enables the motor driver
    
    digitalWrite(PIN_Motor_R_IN1, HIGH);      // HIGH for IN1 indicates forward wheel rotation, LOW indicates reverse wheel rotation
    digitalWrite(PIN_Motor_L_IN1, HIGH);      // HIGH for IN1 indicates forward wheel rotation, LOW indicates reverse wheel rotation
    
    analogWrite(PIN_Right_Motor, speed);      // Runs the right motors at [speed] which is a PWM value from 0-255
    analogWrite(PIN_Left_Motor, speed);       // Runs the left motors at [speed] which is a PWM value from 0-255
  }
  else if (direction.equals("Pan Left")){
    digitalWrite(PIN_Motor_Standby, HIGH);
    
    digitalWrite(PIN_Motor_R_IN1, HIGH);
    digitalWrite(PIN_Motor_L_IN1, LOW);
    
    analogWrite(PIN_Right_Motor, speed);
    analogWrite(PIN_Left_Motor, speed);
  }
  else if (direction.equals("Pan Right")){
    digitalWrite(PIN_Motor_Standby, HIGH);
    
    digitalWrite(PIN_Motor_R_IN1, LOW);
    digitalWrite(PIN_Motor_L_IN1, HIGH);
    
    analogWrite(PIN_Right_Motor, speed);
    analogWrite(PIN_Left_Motor, speed);
  }
  else if (direction.equals("Stop")){
    digitalWrite(PIN_Motor_Standby, LOW);     // Disables the motor driver
  }
}