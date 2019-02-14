#include <QuadratureEncoder.h>
#include <EEPROM.h>

// Motor Pins
#define LEFT_MOTOR_APWM   10
#define LEFT_MOTOR_AIN1   6
#define LEFT_MOTOR_AIN2   5

#define RIGHT_MOTOR_BIN1  7
#define RIGHT_MOTOR_BIN2  8
#define RIGHT_MOTOR_BPWM  9

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
          setKpTurnAngle(serialValue); break;
        case 'i':
          setKiTurnAngle(serialValue); break;
        case 'd': 
          setKdTurnAngle(serialValue); break;
        case 'b': 
          setDB(serialValue); break;
        case 's': 
          setdesiredTurnAngle(serialValue); break;
        case 'e': 
          save2EEPROM(serialValue); break;
      }
      printTurnAngleInfo();
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
  setKpAngle(params[0]);
  setKiAngle(params[1]);
  setKdAngle(params[2]);
}

void save2EEPROM(int crc){
  if(crc==100){
    double params[3]; 
    params[0] = getKpAngle();
    params[1] = getKiAngle();
    params[2] = getKdAngle();
    EEPROM.put(0,params);
    Serial.println("---------------EEPROM Updated----------------");
  }else{
    Serial.println("FAILED EEPROM");
  }
  
}
