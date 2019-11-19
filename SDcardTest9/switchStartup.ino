void switchStarup(){
    readRotSwitch();

  if (rotValue == 10){ //A Automatic mode
    automatic = true;
  }

  else if (rotValue == 11){ //B Timer mode
    timer = true;
  }

  else if (rotValue == 12){ //C, Configure the time for parachute deployment on automatic mode
    while(1){
      readRotSwitch();
      if (rotValue == 0){
        while(1){
          readRotSwitch();
          blinkLED();
          if (previousValue != rotValue){
            EEPROM.write(2, rotValue);
            previousValue = rotValue;
          }
        }
      }
    }   
  }

   else if (rotValue == 13){ //D, Configure the time for parachute deployment on timer mode.
    while(1){
      readRotSwitch();
      if (rotValue == 0){
        while(1){
          readRotSwitch();
          blinkLED();
          if (previousValue != rotValue){
            EEPROM.write(3, rotValue);
            previousValue = rotValue;
          }
        }
      }
    }   
  }
    

  else if (rotValue == 14){ //E, Adjust the servo initial possition
    while(1){
      readRotSwitch();
      myservo.write((14 - rotValue)*180/15);
      blinkLED();
      if (previousValue != rotValue){
        EEPROM.write(0, (14 - rotValue)*180/15);
      }
      previousValue == rotValue;
    }
  }

    if (rotValue == 15){ //F, Adjust the servo final possition
      while(1){
        readRotSwitch();        
        myservo.write(180*rotValue/15); //Work on the problem with the starting possition.
        blinkLED();
        if (previousValue != rotValue){
          EEPROM.write(1, 180*rotValue/15);  
        }
        previousValue == rotValue;
      }
    } 
}

void blinkLED(){
    digitalWrite(A3, HIGH);
    delay(200);
    digitalWrite(A3, LOW);
    delay(200);
}

void readRotSwitch(){
 for (int k = 0; k < 4; k++){
  if (digitalRead(switchPins[k]) == LOW) {
    bitSet(rotValue, k); //sets bit k to 1
  }
  else {
     bitClear(rotValue, k); //sets bit k to 0
  }
 }
}
