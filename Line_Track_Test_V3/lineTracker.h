void lineTrac(){
  drive("STOP", 0, 0);      // Stop all motors

  // Wait 10 microseconds. The delay is required because the motors/motor driver cause interference with the  
  // analogRead, so the motors must be shut off so that an accurate reading can be taken from the line sensors.
  delayMicroseconds(10);    

  // Read the analog values from all 3 line sensors
  leftCurrent = analogRead(PIN_Left_LineTracker);
  rightCurrent = analogRead(PIN_Right_LineTracker);
  middleCurrent = analogRead(PIN_Middle_LineTracker);

  digitalWrite(PIN_Motor_Standby, HIGH);    // Re-enable all motors

  if (leftCurrent > (avgLeftLine + variance) || leftCurrent < (avgLeftLine - variance)) {   // Left tracker detected black tape
    drive("DIFFERENTIAL LEFT", 35, 0);    // Drive straight with a slight left turn
  }
  else if (rightCurrent > (avgRightLine + variance) || rightCurrent < (avgRightLine - variance)) {    // Right tracker detected black tape
    drive("DIFFERENTIAL RIGHT", 0, 35);   // Drive straight with a slight right turn
  }
  else {    // Neither left or right tracker detected black tape
    drive("FORWARD", 35, 35);   // Drive forward
  }
}