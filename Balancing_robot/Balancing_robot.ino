#include <QuadratureEncoder.h>
#include <EEPROM.h>
Encoders leftEncoders(A1,A0);
Encoders rightEncoders(3,4);

float yall,pitch,roll;
double output;

void setup() {
  InitSensors();          // Initialize sensors
  InitMotors();           // Initialize motors
  setupPWM16();
  delay(5000);            // Wait until sensors be ready
  
  // load params from EEPROM
  initEEPROM();
}
char myBuffer[4];
void loop() {

  if(Serial.available()){
    char whichGain = Serial.read();
    if(whichGain >= 'b' && whichGain <= 's'){
        String myString = Serial.readStringUntil('\n');
//        Serial.println(myString);
        for(int i(0); i < 4;i++){
          myBuffer[i] = myString[i];
        }
        double serialValue = atof(myBuffer);
        //Serial.println(mygain);
        switch(whichGain){
        case 'p':
          setKp(serialValue); break;
        case 'i':
          setKi(serialValue); break;
        case 'd': 
          setKd(serialValue); break;
        case 'b': 
          setDB(serialValue); break;
        case 's': 
          setdesiredAngle(serialValue); break;
        case 'e': 
          save2EEPROM(serialValue); break;
      }
    }else{
      Serial.readStringUntil('\n');
    }
  }
  GetIMUReadings();
}

//add dt as parameter?
double getLeftMotorSpeed(){
  static long lastLeftEncoderCount = 0;
  long currentLeftEncoderCount = leftEncoders.getEncoderCount();
  long diff = currentLeftEncoderCount - lastLeftEncoderCount;
  lastLeftEncoderCount = currentLeftEncoderCount;
  double actualLeftSpeed = (double)diff*.928;
  return actualLeftSpeed;
}

double getRightMotorSpeed(){
  static long lastRightEncoderCount = 0;
  long currentRightEncoderCount = rightEncoders.getEncoderCount();
  long diff = currentRightEncoderCount - lastRightEncoderCount;
  lastRightEncoderCount = currentRightEncoderCount;
  double actualRightSpeed = (double)diff*.928;
  return actualRightSpeed;
}

void initEEPROM(){
  double params[3]; //  Kp Ki Kd
  EEPROM.get(0,params);
  setKp(params[0]);
  setKi(params[1]);
  setKd(params[2]);
}

void save2EEPROM(int crc){
  if(crc==100){
    double params[3]; 
    params[0] = getKp();
    params[1] = getKi();
    params[2] = getKd();
    EEPROM.put(0,params);
    Serial.println("---------------EEPROM Updated----------------");
  }else{
    Serial.println("FAILED EEPROM");
  }
  
}
