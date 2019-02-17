#include <QuadratureEncoder.h>
#include <EEPROM.h>

#define RIGHT_MOTOR_BPWM  9
#define LEFT_MOTOR_APWM   10

Encoders leftEncoders(A1, A0);
Encoders rightEncoders(3, 4);

long getAverageEncoderCount() {
  long currentLeftEncoderCount = leftEncoders.getEncoderCount();
  long currentRightEncoderCount = rightEncoders.getEncoderCount();
  return (currentLeftEncoderCount + currentRightEncoderCount) / 2;
}

void initEEPROM() {
  double params[9]; //  first 36 byte from EEPROM is for PID gains
  EEPROM.get(0, params);
  setKpAngle(params[0]);
  setKiAngle(params[1]);
  setKdAngle(params[2]);

  setKpEncoder(params[3]);
  setKiEncoder(params[4]);
  setKdEncoder(params[5]);

  setKpTurnAngle(params[6]);
  setKiTurnAngle(params[7]);
  setKdTurnAngle(params[8]);
}

void setup() {
  initMPU();          // Initialize sensors
  initMotors();           // Initialize motors
  setupPWM16();
  
  // Delay 5 seconds for IMU sensor to stabilize. 
  unsigned long lastTime = millis();
  while(millis() < lastTime+5000);

  // load params from EEPROM
  initEEPROM();
}

void loop() {
  btCommands();
  calcPID();
  printInfo();
}
