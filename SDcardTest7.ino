//Investigate the sampling rate of the accelerometer. What is the lowest sampling rate that I can use so that the sampling speed is not affected?
//The accelerometer is not reliable for measuring angles while the vehicle is moving. This is because the accelerometer measures both the
//acceleration due to gravity and motion.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>
#include <BMI160Gen.h>
Adafruit_BMP280 bme; // I2C

boolean condition = false;
boolean initVar = true;

// Initialization of Kalman Variables
float R[3] = {0.16, 0.004, 9*1e-4};
float Q[3] = {0.16*1e-3, 0.004*1e-2, 9*1e-6};  //Q = process noise covariance, R = measurement noise covariance Larger R means large measurement uncertainty. Larger Q means larger estimation uncertainty. Thus increasing Q corrects more.
double Xpe0[5] = {};  // Xpe0 = prior estimation of signal X at time t=0 (current state)
double Xe1[5] = {};  //Xe1 = estimation of X at time t=1 (previous state)
double Ppe0[5] = {};  //Ppe0 = prior estimation of "error covariance" at t=0,  
double P1[5] = {}, P0[5] = {}; //P1 = error covariance at t=1, P0 = error covariance at t=0
double K[5] = {}, Xe0[5] = {}, Z[5] = {}; //K = Kalman gain, Xe0 = estimation of signal at t=0, Z = measured signal at t=0;
double angle[3] = {};
double oldangle[3] = {};

//static std gx = 0.0624 std gy = 0.0430 std gz = 0.0444 º/s
//static std accy = 0.0041 g;

double alt0;
int a[3] = {};
int gyro[3] = {};
int t1, t2, t3, deltat;

File dataFile;

void setup() {
  
  BMI160.begin(BMI160GenClass::I2C_MODE);
  pinMode(A3, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(10, OUTPUT);
  uint8_t dev_id = BMI160.getDeviceID();
  BMI160.setAccelerometerRange(16); //Accelerometer set to +- 16 g
  BMI160.setGyroRange(500); //Gyroscope set to +- 500 degrees per second
  
  // see if the card is present and can be initialized:
  if (!SD.begin(10)) {
    digitalWrite(A3, HIGH);
    File dataFile = SD.open("datalog.txt", O_CREAT | O_WRITE);
    return;
  }

  if (!bme.begin()) {  
   //digitalWrite(ledPin, HIGH);
   while (1);
  }


  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      dataFile = SD.open(filename, FILE_WRITE);
      break; // leave the loop!
    }
  }


  delay(2000);//Add here a delay so that the user has time to step away from the board
}



void loop() {
dataAcquisition();
  
if (initVar == true){
  oldangle[0] = atan2(a[2], a[1])*180/PI;//Rotation around x (pitch)
  oldangle[2] = atan2(a[0], a[1])*180/PI; //Rotation around z (yaw)
  initVar = false;
}
  
if ((a[1]*(-16.0/32768.0) > 0) && condition == false){
  condition = true;
  digitalWrite(A3, HIGH);
  //digitalWrite(A2, HIGH);
  delay(300);
  digitalWrite(A3, LOW);
  //digitalWrite(A2, LOW);
}

if (dataFile && condition == true){
   logData();
  }
}

void dataAcquisition()
{
 BMI160.readAccelerometer(a[0], a[1], a[2]);
 BMI160.readGyro(gyro[0], gyro[1], gyro[2]);
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

void calculateAngle(){
 for (int k = 0; k < 3; k++){
  angle[k] = oldangle[k] + deltat * Xe0[k+1]/1000.0;
  oldangle[k] = angle[k];
 }
}

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

void logData(){
      for (int k = 1; k < 1000000; k++){ //This k must have an initial value of 1 for stability reasons
      if(k > 1){
        dataAcquisition();
        kalmanFilter();
        calculateAngle();
      }
      else if(k == 1){
        angle[0] = oldangle[0];
        angle[3] = oldangle[3];
        alt0 = Xe0[0] = Xe1[0] = Z[0];
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
      dataFile.println(Xe0[4]); //Acceleration along the y axis
     
      if (k%200 == 0){
        dataFile.flush();
        //digitalWrite(A3, HIGH);
      }
    }
}

