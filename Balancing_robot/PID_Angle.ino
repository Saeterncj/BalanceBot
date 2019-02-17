// PID Variables
// You can change this values to adjust the control
double Kp_Angle ;        // Proportional value 120
double Ki_Angle ;           // Integral value 10
double Kd_Angle ;           // Derivative value 150
double desiredAngle = 0;     // Initial desiredAngle is 0
double lastActualAngle;
void setKpAngle(double KP) {
  Kp_Angle = KP;
  printTiltAngleInfo();
}
void setKiAngle(double KI) {
  Ki_Angle = KI;
  printTiltAngleInfo();
}
void setKdAngle(double KD) {
  Kd_Angle = KD;
  printTiltAngleInfo();
}
double getKpAngle() {
  return Kp_Angle;
}
double getKiAngle() {
  return Ki_Angle;
}
double getKdAngle() {
  return Kd_Angle;
}
// Calculates the PID output
double ComputeAnglePID(double actualAngle)
{
  static double integralError = 0, output;
  double error = desiredAngle - actualAngle;

  //derivative Ki_Angleck
  double dactualAngle = (lastActualAngle - actualAngle);

  // Handle Integral Windup with Clamping
  if (output < resolution && output > -resolution) {
    integralError += error;
  }

  // Compute PID Output
  output = Kp_Angle * error + Ki_Angle * integralError + Kd_Angle * dactualAngle;

  //Saturation
  if (output > resolution) output = resolution;
  else if (output < -(resolution)) output = -(resolution);
  //add setpoint and samplingtime

  lastActualAngle = actualAngle;
  return output;
}

void printTiltAngleInfo() {
  Serial.print("Angle Info: ");
  Serial.print("Kp=");
  Serial.print(Kp_Angle);
  Serial.print(" , ");
  Serial.print("Ki=");
  Serial.print(Ki_Angle);
  Serial.print(" , ");
  Serial.print("Kd=");
  Serial.print(Kd_Angle);
  Serial.print(" , ");
  Serial.print("desired=");
  Serial.print(desiredAngle);
  Serial.print(" , ");
  Serial.print("actual= ");
  Serial.print(lastActualAngle);
  Serial.print("\n");
}
// Seters and geters for desiredAngle
void setDesiredAngle(double d) {
  desiredAngle = d;
}
double getDesiredAngle() {
  return desiredAngle;
}
double getLastAngle(){
  return lastActualAngle;
}
//void incrementTiltAngle(double encoderPID){desiredAngle+=encoderPID;}
double pwmOuts() {
  // get PWM out from
  double pwmOut = 0;
}
