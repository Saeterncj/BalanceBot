// PID Variables
// You can change this values to adjust the control
double Kp_TurnAngle;         // Proportional value 4
double Ki_TurnAngle;           // Integral value 0
double Kd_TurnAngle;           // Derivative value 0
double desiredTurnAngle = 0;     // Initial desiredTurnAngle is 0
double lastActualTurnAngle;
void setKpTurnAngle(double KP) {
  Kp_TurnAngle = KP;
  printTurnAngleInfo();
}
void setKiTurnAngle(double KI) {
  Ki_TurnAngle = KI;
  printTurnAngleInfo();
}
void setKdTurnAngle(double KD) {
  Kd_TurnAngle = KD;
  printTurnAngleInfo();
}
void setDesiredTurnAngle(double d) {
  desiredTurnAngle = d;
}
double getKpTurnAngle() {
  return Kp_TurnAngle;
}
double getKiTurnAngle() {
  return Ki_TurnAngle;
}
double getKdTurnAngle() {
  return Kd_TurnAngle;
}
double getDesiredTurnAngle() {
  return desiredTurnAngle;
}
double getLastTurnAngle(){
  return lastActualTurnAngle;
}
// Calculates the PID output
double ComputeTurnAnglePID(double actualTurnAngle)
{
  static double integralError = 0;
  double error = desiredTurnAngle - actualTurnAngle;
  static double output;

  // Normallize error for heading direction
  if (error  > 180) {
    error -= 360;
    actualTurnAngle += 360;
  }
  if (error < -180) {
    error += 360;
    actualTurnAngle -= 360;
  }
  //derivative Ki_TurnAngleck
  double dactualTurnAngle = (lastActualTurnAngle - actualTurnAngle);

  // for continuous heading direction. 
  if (dactualTurnAngle > 180) {
    dactualTurnAngle -= 360;
    lastActualTurnAngle -= 360;
  }
  if (dactualTurnAngle < -180) {
    dactualTurnAngle += 360;
    lastActualTurnAngle += 360;
  }
  
  // Handle Integral Windup with Clamping
  if (output < resolution && output > -resolution) {
    integralError += error;
  }
  
  // Compute PID Output
  output = Kp_TurnAngle * error + Ki_TurnAngle * integralError + Kd_TurnAngle * dactualTurnAngle;

  //Saturation
  if (output > resolution) output = resolution;
  else if (output < -(resolution)) output = -(resolution);

  lastActualTurnAngle = actualTurnAngle;
  return output;
}

void printTurnAngleInfo() {
  Serial.print("Turn(d) Info: ");
  Serial.print("Kp=");
  Serial.print(Kp_TurnAngle);
  Serial.print(" , ");
  Serial.print("Ki=");
  Serial.print(Ki_TurnAngle);
  Serial.print(" , ");
  Serial.print("Kd=");
  Serial.print(Kd_TurnAngle);
  Serial.print(" , ");
  Serial.print("desired=");
  Serial.print(desiredTurnAngle);
  Serial.print(" , ");
  Serial.print("actual= ");
  Serial.print(lastActualTurnAngle);
  Serial.print("\n");
}
