// PID Variables
// You can change this values to adjust the control
double Kp_TurnAngle = 0;         // Proportional value 120
double Ki_TurnAngle = 0;           // Integral value 10
double Kd_TurnAngle = 0;           // Derivative value 150
double desiredTurnAngle = 0;     // Initial desiredTurnAngle is 0
void setKpTurnAngle(double KP){Kp_TurnAngle = KP;}
void setKiTurnAngle(double KI){Ki_TurnAngle = KI;}
void setKdTurnAngle(double KD){Kd_TurnAngle = KD;}
double getKpTurnAngle(){return Kp_TurnAngle;}
double getKiTurnAngle(){return Ki_TurnAngle;}
double getKdTurnAngle(){return Kd_TurnAngle;}
// Calculates the PID output
double ComputeTurnAnglePID(double actualTurnAngle)
{
  static float lastActualTurnAngle = 0;
  static double integralError = 0;
  double error = desiredTurnAngle - actualTurnAngle;
  
  //derivative Ki_TurnAngleck
  double dactualTurnAngle = (lastActualTurnAngle -actualTurnAngle);
  
  // Handle Integral Windup with Clamping
  if (output < resolution && output > -resolution) {
    integralError += error;
  }

  // Compute PID Output
  output = Kp_TurnAngle * error + Ki_TurnAngle*integralError + Kd_TurnAngle * dactualTurnAngle;
  
  //Saturation
  if (output > resolution) output = resolution;
  else if (output < -(resolution)) output = -(resolution);

  // Remember some variables for next time

  //add setpoint and samplingtime

  lastActualTurnAngle = actualTurnAngle;
  return output;
}

void printTurnAngleInfo(){
  Serial.print("p=");
  Serial.print(Kp_TurnAngle);
  Serial.print(" , ");
  Serial.print("i=");
  Serial.print(Ki_TurnAngle);
  Serial.print(" , ");
  Serial.print("d=");
  Serial.print(Kd_TurnAngle);
  Serial.print(" , ");
  Serial.print("s=");
  Serial.println(desiredTurnAngle);
}

void setdesiredTurnAngle(double d) {desiredTurnAngle = d;}
double GetdesiredTurnAngle() { return desiredTurnAngle; }
