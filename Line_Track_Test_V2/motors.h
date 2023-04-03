/* Drive Modes
 *
 * For non-required parameters, just input an arbitrary value
 *
 * FORWARD:
 *  - The robot will drive straight forward at a given speed, for a given distance
 *  - Required parameters: "direction", "speedL", "distance"
 *  - "speedR" is not required, as it will be set to the same value as "speedL"
 *
 * PAN LEFT:
 *  - The robot will rotate left on the spot
 *  - Required parameters: "direction", "speedL", "distance"
 *  - "speedR" is not required as it will be set to the same value as "speedL"
 *
 * PAN RIGHT:
 *  - The robot will rotate right on the spot
 *  - Required parameters: "direction", "speedL", "distance"
 *  - "speedR" is not required as it will be set to the same value as "speedL"
 *
 * DIFFERENTIAL LEFT:
 *  - The robot will drive forward with a slight left turn
 *  - Required parameters: "direction", "speedL", "distance" 
 *  - "speedR" is not required as it will be calculated based on the differential speed value and "speedL"
 *
 * STOP:
 *  - The robot will not move
 *  - Required parameters: NONE
 */

void drive(String direction, int speedL, int speedR){
  if (direction.equals("FORWARD")){
    speedR = speedL;

    digitalWrite(PIN_Motor_Standby, HIGH);    // Enables the motor driver
    
    digitalWrite(PIN_Motor_L_IN1, HIGH);      // HIGH for IN1 indicates forward wheel rotation, LOW indicates reverse wheel rotation
    digitalWrite(PIN_Motor_R_IN1, HIGH);      // HIGH for IN1 indicates forward wheel rotation, LOW indicates reverse wheel rotation
    
    analogWrite(PIN_Left_Motor, speedL);       // Runs the left motors at [speed] which is a PWM value from 0-255
    analogWrite(PIN_Right_Motor, speedR);      // Runs the right motors at [speed] which is a PWM value from 0-255
  }

  else if (direction.equals("PAN LEFT")){
    speedR = speedL;

    digitalWrite(PIN_Motor_Standby, HIGH);
    
    digitalWrite(PIN_Motor_L_IN1, LOW);
    digitalWrite(PIN_Motor_R_IN1, HIGH);
    
    analogWrite(PIN_Left_Motor, speedL);
    analogWrite(PIN_Right_Motor, speedR);
  }

  else if (direction.equals("PAN RIGHT")){
    speedR = speedL;

    digitalWrite(PIN_Motor_Standby, HIGH);
    
    digitalWrite(PIN_Motor_L_IN1, HIGH);
    digitalWrite(PIN_Motor_R_IN1, LOW);

    analogWrite(PIN_Left_Motor, speedL);
    analogWrite(PIN_Right_Motor, speedR);
  }

  else if (direction.equals("DIFFERENTIAL LEFT")){
    speedR = speedL + 10;

    digitalWrite(PIN_Motor_Standby, HIGH);

    digitalWrite(PIN_Motor_L_IN1, HIGH);
    digitalWrite(PIN_Motor_R_IN1, HIGH);

    analogWrite(PIN_Left_Motor, speedL);
    analogWrite(PIN_Right_Motor, speedR);
  }

  else if (direction.equals("DIFFERENTIAL RIGHT")){
    speedL = speedR + 10;

    digitalWrite(PIN_Motor_Standby, HIGH);

    digitalWrite(PIN_Motor_L_IN1, HIGH);
    digitalWrite(PIN_Motor_R_IN1, HIGH);

    analogWrite(PIN_Left_Motor, speedL);
    analogWrite(PIN_Right_Motor, speedR);
  }

  else if (direction.equals("STOP")){
    digitalWrite(PIN_Motor_Standby, LOW);     // Disables the motor driver
    speedL = 0;
    speedR = 0;
  }
}