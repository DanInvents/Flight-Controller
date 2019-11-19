void dataAcquisition()
{
 BMI160.readAccelerometer(a[0], a[1], a[2]);
 BMI160.readGyro(gyro[0], gyro[1], gyro[2]);
 T = BMI160.readTemperature();
 Z[0] = bme.readAltitude(1013.25);

 for (int k = 0; k < 4; k++){
  if (k < 3){
    Z[k+1] =  gyro[k]*500.0/32768.0;
  }

  else if (k == 3){
    Z[k+1] = -a[1]*16.0/32768.0;
  }
 }
}
