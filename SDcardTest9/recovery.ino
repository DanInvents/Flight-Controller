void recovery(){ 
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
}
