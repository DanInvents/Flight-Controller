// This program performs a Kalman filter of the flight data. It smoothens the data and ignores transitory events.

// Q = process noise covariance
// R = measurement noise covariance. Larger R means large measurement uncertainty. Larger Q means larger estimation uncertainty. Thus increasing Q corrects more.
// Xpe0 = prior estimation of signal X at time t=0 (current state)
// Xe1 = estimation of X at time t=1 (previous state)
// Ppe0 = prior estimation of "error covariance" at t=0,  
// P1 = error covariance at t=1, P0 = error covariance at t=0
// K = Kalman gain, Xe0 = estimation of signal at t=0, Z = measured signal at t=0;

void kalmanFilter(){
  for (int k=0; k<4; k++){
    Xpe0[k] = Xe1[k]; // Assumption of prediction 1
    
    if (k == 0){ //Kalman filter parameters for altitude estimation
    Ppe0[k] = P1[k] + Q[0]; // Update of prior estimation of "error covariance"
    K[k] = Ppe0[k]/(Ppe0[k] + R[0]); // Measurement update or correction of "Kalman gain"
    }
    else if (k > 0){ // Kalman filter parameters for rotation rate estimation
    Ppe0[k] = P1[k] + Q[1]; // Update of prior estimation of "error covariance"
    K[k] = Ppe0[k]/(Ppe0[k] + R[1]); // Measurement update or correction of "Kalman gain"      
    }
    
    Xe0[k] = Xpe0[k] + K[k] * (Z[k] - Xpe0[k]); // Measurement update or correction of "estimated signal"
    P0[k] = (1 - K[k]) * Ppe0[k]; // Measurement update or correction of "error covariance";
    Xe1[k] = Xe0[k];
    P1[k] = P0[k];
  }
  return;
}
