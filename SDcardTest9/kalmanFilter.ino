void kalmanFilter(){
  for (int k=0; k<5; k++){
    Xpe0[k] = Xe1[k]; //Assumption of prediction 1
    
    if (k == 0){
    Ppe0[k] = P1[k] + Q[0]; //Assumption of prediction 2
    K[k] = Ppe0[k]/(Ppe0[k] + R[0]); // Measurement update or correction of "Kalman gain"
    }
    else if (k < 4){
    Ppe0[k] = P1[k] + Q[1]; //Assumption of prediction 2
    K[k] = Ppe0[k]/(Ppe0[k] + R[1]); // Measurement update or correction of "Kalman gain"      
    }
    else if (k == 4){
    Ppe0[k] = P1[k] + Q[2]; //Assumption of prediction 2
    K[k] = Ppe0[k]/(Ppe0[k] + R[2]); // Measurement update or correction of "Kalman gain"   
    }
    
    Xe0[k] = Xpe0[k] + K[k] * (Z[k] - Xpe0[k]); // Measurement update or correction of "estimated signal"
    P0[k] = (1 - K[k]) * Ppe0[k]; // Measurement update or correction of "error covariance";
    Xe1[k] = Xe0[k];
    P1[k] = P0[k];
  }
  return;
}
