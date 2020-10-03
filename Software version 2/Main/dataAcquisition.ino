// This program reads the flight parameters and stores them in arrays.

void dataAcquisition()
{
 BMI160.readAccelerometer(a[0], a[1], a[2]);  // Read acceleration along the x, y and z axes
 BMI160.readGyro(gyro[0], gyro[1], gyro[2]);  // Read rotation rate about the x, y and z axes
 T = BMI160.readTemperature();  // Read temperature

 for (int k = 0; k < 4; k++){
  if (k == 0){
    Z[0] = bme.readAltitude(1013.25); // Convert pressure to altitude
  }
  
  if (0 < k < 3){
    Z[k+1] =  gyro[k]*500.0/32768.0;  // Convert angular rate to degrees per second
  }

  else if (k == 3){
    Z[k+1] = -a[1]*16.0/32768.0;  // Convert acceleration to g's
  }
 }
}
