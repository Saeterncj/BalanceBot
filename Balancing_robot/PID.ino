
// PID variables
int outMax = resolution;
int outMin = -resolution;
float lastactualAngle = 0;
double integralError = 0;

// PID constants
// You can change this values to adjust the control
double kp ;         // Proportional value 120
double ki ;           // Integral value 10
double kd ;           // Derivative value 150
//double output;
double desiredAngle = 0;     // Initial desiredAngle is 0
int deadBand = 0;
void setKp(double KP){kp = KP;}
void setKi(double KI){ki = KI;}
void setKd(double KD){kd = KD;}
double getKp(){return kp;}
double getKi(){return ki;}
double getKd(){return kd;}
void setDB(int db){deadBand=db;}
// Calculates the PID output
double Compute(double actualAngle)
{
  double error = desiredAngle - actualAngle;

  //derivative kick
  double dactualAngle = (lastactualAngle -actualAngle);

  // Handle Integral Windup with Clamping
  if (output < resolution && output > -resolution) {
    integralError += error;
  }

  // Compute PID Output
  output = kp * error + ki*integralError + kd * dactualAngle;

  // Include deadband
  if(error > 0) output+=deadBand;
  else output-=deadBand;
  
  //Saturation
  if (output > resolution) output = resolution;
  else if (output < -(resolution)) output = -(resolution);

  // Remember some variables for next time
//  Serial.print(actualAngle);
//  Serial.print(" , ");
//  Serial.println(output);
  Serial.print("p=");
  Serial.print(kp);
  Serial.print(" , ");
  Serial.print("i=");
  Serial.print(ki);
  Serial.print(" , ");
  Serial.print("d=");
  Serial.print(kd);
  Serial.print(" , ");
  Serial.print("s=");
  Serial.print(desiredAngle);
  Serial.print(" , ");
  Serial.print("b=");
  Serial.println(deadBand);
  //add setpoint and samplingtime

  lastactualAngle = actualAngle;
  return output;
}

void findMotorDeadBand() {
  rightEncoders.setEncoderCount(0L);
  leftEncoders.setEncoderCount(0L);
  int pwmVal = 1;
  long start = millis();
  while (leftEncoders.getEncoderCount() == 0 && rightEncoders.getEncoderCount()==0) {
    MotorControl(pwmVal);
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
void setdesiredAngle(double d) {

  desiredAngle = d;

}

double GetdesiredAngle() {

  return desiredAngle;

}
