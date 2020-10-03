// DISCLAIMER

//Software version 2. Release date: 03.10.2020

// In no respect shall DanInvents be accountable for any liabilities, claims, demands, damages or suits resulting from the use of
// the flight controller and/or this software. By using this software, you assume all risks associated with this product and
// its associated features. While the circuitry and software have been tested, they should be considered experimental and handled
// with caution.

// Before uploading this code make sure that you have downloaded the modified Adafruit libraries.
// You will also need the EmotiBit BMI160 library for the accelerometer/gyroscope IC.
// Thanks to Adafruit, EmotiBit for the libraries and also to Homemade Multibody Dynamics for a guide into how to log data fast
// with an Arduino. I would also like to thank MartinMcC for showing how to use a rotary encoder with Arduino.
// Special thanks to Barun Basnet for the exceptional work on Kalman filters.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>
#include <BMI160Gen.h> //
#include <Servo.h>
#include <EEPROM.h>
Adafruit_BMP280 bme; // I2C
Servo myservo;

byte switchPins[4] = {8, 6, 5, 7}; //Digital pins assigned to the rotary switch
byte rotValue = B0000; // Variable for printing value over serial debug
byte switchPos; // Variable for storing the current switch possition
byte previousValue; //Variable for storing the previous switch possition

//Boolean variables defining the state of the program
bool initVar = true;
bool deploy = false;
bool automatic = false;
bool timer = false;

// Initialization of Kalman Variables
float R[2] = {0.16, 0.004}; //R = measurement noise covariance. Larger R means large measurement uncertainty
float Q[2] = {0.16*1e-3, 0.004*1e-2};  //Q = process noise covariance. Larger Q means larger estimation uncertainty. Thus increasing Q corrects more
double Xpe0[4] = {};  // Xpe0 = prior estimation of signal X at time t=0 (current state)
double Xe1[4] = {};  //Xe1 = estimation of X at time t=1 (previous state)
double Ppe0[4] = {};  //Ppe0 = prior estimation of "error covariance" at t=0
double P1[4] = {}, P0[4] = {}; //P1 = error covariance at t=1, P0 = error covariance at t=0
double K[4] = {}, Xe0[4] = {}, Z[4] = {}; //K = Kalman gain, Xe0 = estimation of signal at t=0, Z = measured signal at t=0
double angle[3] = {}; //Current pitch, yaw and roll angles
double oldangle[3] = {}; //Previous pitch, yaw and roll angles
float altold; //Baseline pressure
float altold2;
int a[3] = {}; //Acceleration vector
int gyro[3] = {}; //Rotation rate vector
long int t1, t2, t3; //Time variables
int tconfig;
int deltat; //Time step of every loop iteration
int T; //Temperature value
int q;

File dataFile; //Creates a dataFile object

void setup() {
  
  myservo.attach(A0); //Initializes servo
  myservo.write(EEPROM.read(0)); //Moves the servo to the initial possition

  //Here we pull up all the rotary switch pins
  for (int i = 0; i < 4; i = i + 1){
    pinMode(switchPins[i], INPUT_PULLUP);
  }
  
  BMI160.begin(BMI160GenClass::I2C_MODE); //Initialize the accelerometer/gyroscope in I2C mode
  bme.begin(); //Initialize the barometer

  pinMode(A2, OUTPUT); //Buzzer pin
  pinMode(A3, OUTPUT); //Blue LED pin
  pinMode(2, INPUT_PULLUP); //Trigger pin definition
  pinMode(4, OUTPUT); //Utility pin definition
  pinMode(10, OUTPUT); //SD card pin
  
  uint8_t dev_id = BMI160.getDeviceID(); //Get the accelerometer address
  BMI160.setAccelerometerRange(16); //Accelerometer set to +- 16 g
  BMI160.setGyroRange(500); //Gyroscope set to +- 500 degrees per second
  BMI160.setAccelerometerRate(200); //Accelerometer sampling frequency set to 200 Hz
  BMI160.setGyroRate(200);  //Gyro sampling frequency set to 200 Hz
  delay(3000);//This delay allows the user to step away from the board

  switchStartup(); //Read the rotary switch and select the operation mode
  SDstartup();  //Initialize the SD card
}

void loop() {
  dataAcquisition(); //Acquire data of the initial conditions
  kalmanFilter(); //Initializes the Kalman filter parameters

  if (initVar == true){ //Initialize variables and beep before take off.
    q++;
    oldangle[0] = atan2(a[2], a[1])*180/PI;//Rotation around x (pitch)
    oldangle[2] = atan2(a[0], a[1])*180/PI; //Rotation around z (yaw)
    
    if (q == 400){
      digitalWrite(A2, HIGH);
    }
    else if (q == 450){
      digitalWrite(A2, LOW);
      q = 0;
    }
  }

  if ((-a[1]*16.0/32768.0 > -0.2) || digitalRead(2) == HIGH){ //Launch detection condition.
    initVar = false;
    digitalWrite(A2, LOW);
    if (dataFile){
      logData();
    }
  }
}
