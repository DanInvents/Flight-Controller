void logData(){
  while(1){
    if(k > 1){
      dataAcquisition();
      kalmanFilter();
      calculateAngle();
    }
    else if(k == 1){
      angle[0] = oldangle[0];
      angle[3] = oldangle[3];
      alt0 = altold = Xe0[0] = Xe1[0] = Z[0];
      Xe0[4] = Xe1[4] = Z[4]; 
      t1 = millis();
      t3 = t1;
    }
      
  t2 = millis();
  deltat = t2 - t3;
  t3 = t2;
  dataFile.print(t2-t1);
  dataFile.print(',');
  dataFile.print(Xe0[0]- alt0);
  dataFile.print(',');
  dataFile.print(angle[0]); //Pitch
  dataFile.print(',');
  dataFile.print(angle[1]); //Roll  
  dataFile.print(',');
  dataFile.print(angle[2]); //Yaw
  dataFile.print(',');
  dataFile.print(Xe0[4]); //Acceleration along the y axis
  dataFile.print(',');
  dataFile.println(23.0 + T*pow(0.5,9));
  recovery();
  
  if (k%500 == 0){
    digitalWrite(A2, HIGH);
    dataFile.flush();
    k = 1;
    digitalWrite(A2, LOW);
  }

  k++;
  }
}

void calculateAngle(){
 for (int k = 0; k < 3; k++){
  angle[k] = oldangle[k] + deltat * Xe0[k+1]/1000.0;
  oldangle[k] = angle[k];
 }
}
