//Investigate the sampling rate of the accelerometer. What is the lowest sampling rate that I can use so that the sampling speed is not affected?
//The accelerometer is not reliable for measuring angles while the vehicle is moving. This is because the accelerometer measures both the
//acceleration due to gravity and motion.
//Look into the timing issue.
//The readings from the switch are too unstable to have a large time period. 

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>
#include <BMI160Gen.h>
#include <Servo.h>
#include <EEPROM.h>
Adafruit_BMP280 bme; // I2C
Servo myservo;

byte switchPins[4] = {8, 6, 5, 7};

byte test = B0000; // Variable for printing value over serial debug
byte switchPos; // Variable for storing the switch possition.
int tconfig;
byte previousValue;

boolean condition = false;
boolean initVar = true;
bool final = false;
bool initial = false;
bool deploy = false;
bool automatic = false;
bool timer = false;

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
double altold;
int a[3] = {};
int gyro[3] = {};
int t1, t2, t3, deltat;
int n;
int m;
int mold;

File dataFile;

void setup() {
  
  myservo.attach(A0);
  
  for (int i = 0; i < 4; i = i + 1){
    pinMode(switchPins[i], INPUT_PULLUP);
  }
  
  BMI160.begin(BMI160GenClass::I2C_MODE);
  bme.begin();
  pinMode(A3, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(4, OUTPUT);
  uint8_t dev_id = BMI160.getDeviceID();
  BMI160.setAccelerometerRange(16); //Accelerometer set to +- 16 g
  BMI160.setGyroRange(500); //Gyroscope set to +- 500 degrees per second
  
  // see if the card is present and can be initialized:
  if (!SD.begin(10)) {
    digitalWrite(A3, HIGH);
    while(1);
  }

  char filename[] = "L00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[1] = i/10 + '0';
    filename[2] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      dataFile = SD.open(filename, O_CREAT | O_WRITE);
      break; // leave the loop!
    }
  }
  myservo.write(EEPROM.read(0));
  delay(2000);//Add here a delay so that the user has time to step away from the board
}



void loop() {
startup();
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

void readRotSwitch(){
 for (int k = 0; k < 4; k++){
  if (digitalRead(switchPins[k]) == LOW) {
    bitSet(test, k); //sets bit k to 1
  }
  else {
     bitClear(test, k); //sets bit k to 0
       }
  }
}

void logData(){
      for (int k = 1; k < 1000000; k++){ //This k must have an initial value of 1 for stability reasons
        readRotSwitch();
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
      dataFile.print(test);
      dataFile.print(',');  
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

      if (automatic == true){
        if ((Xe0[0] - altold) < -0.02){
         n++;
         }
        if (n > 4 & deploy == false){
         digitalWrite(A3, HIGH);
         deploy = true;
         tconfig = millis() - t1;
         }
        else if ((Xe0[0] - altold) >= 0){
        n = 0;
        }
         altold = Xe0[0];
      }


      if (deploy == true & t2-t1-tconfig > 600 * EEPROM.read(2)){
        myservo.write(EEPROM.read(1));
      }

      if (timer == true & t2-t1-tconfig > 1000 * EEPROM.read(3)){
        myservo.write(EEPROM.read(1));
      }
      
      if (k%200 == 0){
        digitalWrite(4, HIGH);
        dataFile.flush();
        digitalWrite(4, LOW);
      }
      
    }
}

void startup(){
    readRotSwitch();
    
    if (test == 15){ //Adjust the servo final possition
      final = true;
      while(final){
        readRotSwitch();        
        digitalWrite(A3, HIGH);
        delay(300);
        digitalWrite(A3, LOW);
        delay(300);
        myservo.write(180*test/15); //Work on the problem with the starting possition.
        if (previousValue != test){
          EEPROM.write(1, 180*test/15);  
        }
        
        previousValue == test;
      }
    }

  if (test == 14){ //Adjust the servo initial possition
    initial = true;
    while(initial){
      readRotSwitch();
      digitalWrite(A3, HIGH);
      delay(300);
      digitalWrite(A3, LOW);
      delay(300);
      myservo.write((14 - test)*180/15);
      if (previousValue != test){
        EEPROM.write(0, (14 - test)*180/15);
      }
      previousValue == test;
    }
  }

  if (test == 12){
    while(1){
      readRotSwitch();
      if (test == 0){
        while(1){
          readRotSwitch();
          if (previousValue != test){
            EEPROM.write(2, test);
            previousValue = test;
          }
        }
      }
    }   
  }

    if (test == 13){
    while(1){
      readRotSwitch();
      if (test == 0){
        while(1){
          readRotSwitch();
          if (previousValue != test){
            EEPROM.write(3, test);
            previousValue = test;
          }
        }
      }
    }   
  }
  
    if (test == 10){
      automatic = true;
    }

    if (test == 11){
      timer = true;
    }
  }
