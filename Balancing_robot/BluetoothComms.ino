bool printFlag = false;

char myBuffer[5];

void btCommands() {
  if (Serial.available()) {
    char whichCommand = Serial.read();
    if ((whichCommand >= 'A' && whichCommand <= 'Z') || (whichCommand >= 'a' && whichCommand <= 'z')) {
      String myString = Serial.readStringUntil('\n');
      for (int i(0); i < 5; i++) {
        myBuffer[i] = myString[i];
      }
      double serialValue = atof(myBuffer);
      //Serial.println(mygain);
      switch (whichCommand) {
        case 'T':
        case 't':
          setKpTurnAngle(serialValue);break;
        case 'U':
        case 'u':
          setKiTurnAngle(serialValue);break;
        case 'R':
        case 'r':
          setKdTurnAngle(serialValue); break;
        case 'D':
        case 'd':
          setDesiredTurnAngle(serialValue);break;
          
        case 'A':
        case 'a':
          setKpAngle(serialValue);break;
        case 'N':
        case 'n':
          setKiAngle(serialValue);break;
        case 'G':
        case 'g':
          setKdAngle(serialValue); break;
        case 'L':
        case 'l':
          setDesiredAngle(serialValue);break;
          
        case 'P':
        case 'p':
          setKpEncoder(serialValue);break;
        case 'O':
        case 'o':
          setKiEncoder(serialValue);break;
        case 'S':
        case 's':
          setKdEncoder(serialValue);break;
        case 'I':
        case 'i':
          setDesiredEncoder(serialValue);break;
          
        case 'b':
          setDB(serialValue);break;
        case 'E':
        case 'e':
          save2EEPROM(serialValue); break;
        case 'F':
        case 'f':
          setPrintFlag(serialValue); break;
      }
    } else {
      Serial.readStringUntil('\n');
    }
  }
}

void save2EEPROM(int crc) {
  double params[3];
  int location = 0;
  switch (crc) {
    case 123:
      params[0] = getKpAngle();
      params[1] = getKiAngle();
      params[2] = getKdAngle();
      break;
    case 456:
      params[0] = getKpEncoder();
      params[1] = getKiEncoder();
      params[2] = getKdEncoder();
      location = 1;
      break;
    case 789:
      params[0] = getKpTurnAngle();
      params[1] = getKiTurnAngle();
      params[2] = getKdTurnAngle();
      location = 2;
      break;
    default:
      Serial.println("FAILED EEPROM"); return;
  }
  //uses EEprom update, so it doesnt write if values doesnt change
  EEPROM.put(location * sizeof(params), params);
  Serial.println("---------------EEPROM Updated----------------");
}

void setPrintFlag(int whichFlag) {
  if(whichFlag ==1){
    printFlag = (printFlag) ? false : true;
  }else if(whichFlag ==2){
    printFlag = false;
    printTiltAngleInfo();
    printEncoderInfo();
    printTurnAngleInfo();
  } 
}

void printInfo() {
  static unsigned long lastMilli = 0;
  if (printFlag && millis() - lastMilli > 100) {
    lastMilli = millis();
    // Print whiichever Info is enabled
    //Serial.print(getDesiredAngle());
    Serial.print("Angle: ");
    Serial.print(getDesiredAngle());
    Serial.print(" , ");
    Serial.print(getLastAngle());
    Serial.print(" Position: ");
    Serial.print(getDesiredEncoder());
    Serial.print(" , ");
    Serial.print(getLastEncoder());
    Serial.print(" Turn: ");
    Serial.print(getDesiredTurnAngle());
    Serial.print(" , ");
    Serial.print(getLastTurnAngle());
    Serial.print("\n");
  }
}
