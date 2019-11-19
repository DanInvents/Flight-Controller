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

byte rotValue = B0000; // Variable for printing value over serial debug
byte switchPos; // Variable for storing the switch possition.
byte previousValue;

boolean trigger = false;
boolean initVar = true;
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
long int t1, t2, t3, tconfig;
int deltat;
int n, m, mold;
int T;
int k = 1;

File dataFile;

void setup() {
  
  myservo.attach(A0);
  
  for (int i = 0; i < 4; i = i + 1){
    pinMode(switchPins[i], INPUT_PULLUP);
    pinMode(2, INPUT_PULLUP);
  }
  
  BMI160.begin(BMI160GenClass::I2C_MODE);
  bme.begin();
  pinMode(A3, OUTPUT);
  pinMode(A2, OUTPUT); //Buzzer pin
  pinMode(10, OUTPUT);
  pinMode(4, OUTPUT);
  uint8_t dev_id = BMI160.getDeviceID();
  BMI160.setAccelerometerRange(16); //Accelerometer set to +- 16 g
  BMI160.setGyroRange(500); //Gyroscope set to +- 500 degrees per second
  BMI160.setAccelerometerRate(200);
  BMI160.setGyroRate(200);
  
  
  // see if the card is present and can be initialized:
  if (!SD.begin(10)) {
    digitalWrite(A3, HIGH);
    while(1);
  }

  char filename[] = "00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[0] = i/10 + '0';
    filename[1] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      dataFile = SD.open(filename, O_CREAT | O_WRITE);
      break; // leave the loop!
    }
    else if (SD.exists(F("99.CSV"))){
      while(1){
        digitalWrite(A3, HIGH);
      }
    }
  }
  dataFile.println(F("Time (ms), Altitude (m), Pitch (deg), Roll (deg), Yaw (deg), Accy (g), Temp (C)"));
  dataFile.flush();
  myservo.write(EEPROM.read(0));
  delay(2000);//Add here a delay so that the user has time to step away from the board
}



void loop() {
  switchStarup();
  dataAcquisition();
  
if (initVar == true){
  oldangle[0] = atan2(a[2], a[1])*180/PI;//Rotation around x (pitch)
  oldangle[2] = atan2(a[0], a[1])*180/PI; //Rotation around z (yaw)
  initVar = false;
}
  
if (((a[1]*(-16.0/32768.0) > 0) || digitalRead(2) == HIGH) && trigger == false){
  trigger = true;
  digitalWrite(4, HIGH);
}

if (dataFile && trigger == true){
    logData();
  }
}
