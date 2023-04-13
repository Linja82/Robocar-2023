void calibrate() {
  float leftReadings[10];
  float middleReadings[10];
  float rightReadings[10];
  
  for (int i = 0; i < 10; i++) {  // Read 10 values from each line sensor
    leftReadings[i] = analogRead(PIN_Left_LineTracker);
    middleReadings[i] = analogRead(PIN_Middle_LineTracker);
    rightReadings[i] = analogRead(PIN_Right_LineTracker);
    
    delay(200);
  }
  
  float leftSum, middleSum, rightSum;
  
  for (int i = 0; i < 10; i++) {  // sum the line sensor values previously collected
    leftSum += leftReadings[i];
    middleSum += middleReadings[i];
    rightSum += rightReadings[i];
  }
  
  avgLeftLine = leftSum / 10.0;       // Should be the floor colour 
  avgMiddleLine = middleSum / 10.0;   // Should be black tape
  avgRightLine = rightSum / 10.0;     // Should be the floor colour 
}