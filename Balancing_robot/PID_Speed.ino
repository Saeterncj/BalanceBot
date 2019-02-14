// PID Variables
// You can change this values to adjust the control
double Kp_Speed = 5;         // Proportional value 120
double Ki_Speed = 0;           // Integral value 10
double Kd_Speed = 0;           // Derivative value 150
double desiredSpeed = 0;     // Initial desiredSpeed is 0
void setKpSpeed(double KP){Kp_Speed = KP;}
void setKiSpeed(double KI){Ki_Speed = KI;}
void setKdSpeed(double KD){Kd_Speed = KD;}
double getKpSpeed(){return Kp_Speed;}
double getKiSpeed(){return Ki_Speed;}
double getKdSpeed(){return Kd_Speed;}
// Calculates the PID output
double ComputeSpeedPID(double actualSpeed)
{
  static float lastActualSpeed = 0;
  static double integralError = 0;
  double error = desiredSpeed - actualSpeed;
  
  //derivative Ki_Speedck
  double dactualSpeed = (lastActualSpeed -actualSpeed);
  
  // Handle Integral Windup with Clamping
  if (output < resolution && output > -resolution) {
    integralError += error;
  }

  // Compute PID Output
  output = Kp_Speed * error + Ki_Speed*integralError + Kd_Speed * dactualSpeed;
  
  //Saturation
  if (output > resolution) output = resolution;
  else if (output < -(resolution)) output = -(resolution);

  // Remember some variables for next time
//  Serial.print("p=");
//  Serial.print(Kp_Speed);
//  Serial.print(" , ");
//  Serial.print("i=");
//  Serial.print(Ki_Speed);
//  Serial.print(" , ");
//  Serial.print("d=");
//  Serial.print(Kd_Speed);
//  Serial.print(" , ");
//  Serial.print("s=");
//  Serial.println(desiredSpeed);
  //add setpoint and samplingtime

  lastActualSpeed = actualSpeed;
  return output;
}
