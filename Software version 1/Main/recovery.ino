// This program operates the servo motor.
int n;


void recovery(){ 
  
  if (automatic == true){ // In automatic mode apogee is detected when the altitude decreases in 4 consecutive points
    if ((Xe0[0] - altold) < (-0.01)){
      n++;
     }
    if (n > 4 & deploy == false){
     deploy = true;
     tconfig = millis() - t2;
     }
    else if ((Xe0[0] - altold) >= 0){
      n = 0;
    }
     altold = Xe0[0];
  }
  
  
  if (deploy == true & t2-t1-tconfig > 600 * EEPROM.read(2)){ // The deploy time in automatic mode can be adjusted in steps of 600 ms
    myservo.write(EEPROM.read(1));
    digitalWrite(A3, HIGH); // The blue LED turns on
    digitalWrite(4, HIGH); // The utility pin goes HIGH
  }
  
  if (timer == true & t2-t1-tconfig > 1000 * EEPROM.read(3)){ // The deploy time in timer mode can be adjusted in steps of 1000 ms
    myservo.write(EEPROM.read(1));
    digitalWrite(A3, HIGH); // The blue LED turns on
    digitalWrite(4, HIGH); // The utility pin goes HIGH
  }
}
