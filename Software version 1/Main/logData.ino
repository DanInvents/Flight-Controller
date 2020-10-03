// This program logs data to the SD card and calculates the pitch, yaw and roll angles.
int k = 1;

void logData(){
  while(1){
    
    if(k == 1){ // Setting the time and variables at launch
      t1 = millis();
      t3 = t2 = t1;
      angle[0] = oldangle[0];
      angle[2] = oldangle[2];
      alt0 = altold = Xe0[0] = Xe1[0] = Z[0];
      Xe0[4] = Xe1[4] = Z[4]; 
    }

     else if(k > 1){ // Updating the variables after launch has been detected
      t2 = millis();
      recovery();
      dataAcquisition();
      kalmanFilter();
      calculateAngle();
    }
      
  deltat = t2 - t3;
  t3 = t2;
  dataFile.print(t2-t1); // Time after take-off has been detected
  dataFile.print(',');
  dataFile.print(Xe0[0]- alt0); // Relative altitude to the starting possition
  dataFile.print(',');
  dataFile.print(angle[0]); // Pitch
  dataFile.print(',');
  dataFile.print(angle[1]); // Roll  
  dataFile.print(',');
  dataFile.print(angle[2]); // Yaw
  dataFile.print(',');
  dataFile.print(Xe0[4]); // Acceleration along the y axis
  dataFile.print(',');
  dataFile.println(23.0 + T*pow(0.5,9)); // Temperature in ºC

  if (k == 450){
   digitalWrite(A2, HIGH); // Piezo buzzer starts beeping
  }
  
  else if (k == 500){
    dataFile.flush(); // Write data to SD card
    k = 1;
    digitalWrite(A2, LOW);  // Piezo buzzer stops beeping
  }
  
  k++;
  }
}

void calculateAngle(){ // Update the pitch, yaw and roll angles from the rotational rates
 for (int j = 0; j < 3; j++){
  angle[j] = oldangle[j] + deltat * Xe0[j+1]/1000.0;
  oldangle[j] = angle[j];
 }
}
