// PID Variables
// You can change this values to adjust the control
double Kp_Angle ;         // Proportional value 120
double Ki_Angle ;           // Integral value 10
double Kd_Angle ;           // Derivative value 150
double desiredAngle = 0;     // Initial desiredAngle is 0
void setKpAngle(double KP){Kp_Angle = KP;}
void setKiAngle(double KI){Ki_Angle = KI;}
void setKdAngle(double KD){Kd_Angle = KD;}
double getKpAngle(){return Kp_Angle;}
double getKiAngle(){return Ki_Angle;}
double getKdAngle(){return Kd_Angle;}
// Calculates the PID output
double ComputeAnglePID(double actualAngle)
{
  static float lastactualAngle = 0;
  static double integralError = 0;
  double error = desiredAngle - actualAngle;
  
  //derivative Ki_Angleck
  double dactualAngle = (lastactualAngle -actualAngle);
  
  // Handle Integral Windup with Clamping
  if (output < resolution && output > -resolution) {
    integralError += error;
  }

  // Compute PID Output
  output = Kp_Angle * error + Ki_Angle*integralError + Kd_Angle * dactualAngle;
  
  //Saturation
  if (output > resolution) output = resolution;
  else if (output < -(resolution)) output = -(resolution);

  // Remember some variables for next time
//  Serial.print("p=");
//  Serial.print(Kp_Angle);
//  Serial.print(" , ");
//  Serial.print("i=");
//  Serial.print(Ki_Angle);
//  Serial.print(" , ");
//  Serial.print("d=");
//  Serial.print(Kd_Angle);
//  Serial.print(" , ");
//  Serial.print("s=");
//  Serial.println(desiredAngle);
  //add setpoint and samplingtime

  lastactualAngle = actualAngle;
  return output;
}


int deadBand = 0;
void setDB(int db){deadBand=db;}
void findMotorDeadBand() {
  rightEncoders.setEncoderCount(0L);
  leftEncoders.setEncoderCount(0L);
  int pwmVal = 1;
  long start = millis();
  while (leftEncoders.getEncoderCount() == 0 && rightEncoders.getEncoderCount()==0) {
    MotorControl(RIGHT_MOTOR_BPWM, pwmVal);
    MotorControl(LEFT_MOTOR_APWM, pwmVal);
    if (millis() - start > 40) {
      start = millis();
      pwmVal++;
    }
  }
  deadBand = pwmVal;
  Serial.print("The value for Deadband is: ");
  Serial.println(deadBand);
}

// Seters and geters for desiredAngle
void setdesiredAngle(double d) {desiredAngle = d;}
double GetdesiredAngle() { return desiredAngle; }

double pwmOuts(){
  // get PWM out from 
  double pwmOut = 0;
}
