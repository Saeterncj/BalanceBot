// PID Variables
#define MAX_ANGLE_FORWARD 4
#define MAX_ANGLE_BACKWARD 5
// You can change this values to adjust the control
double Kp_Encoder ;         // Proportional value 0.054
double Ki_Encoder ;           // Integral value
double Kd_Encoder ;           // Derivative value carpet = 0
double desiredEncoder = 0;     // Initial desiredEncoder is 0
double incremental = 2;
double userDesiredEncoder = 0;
double lastActualEncoder = 0;

void setKpEncoder(double KP) {
  Kp_Encoder = KP;
  printEncoderInfo();
}
void setKiEncoder(double KI) {
  Ki_Encoder = KI;
  printEncoderInfo();
}
void setKdEncoder(double KD) {
  Kd_Encoder = KD;
  printEncoderInfo();
}
void setDesiredEncoder(double EncoderCount) {
  userDesiredEncoder = EncoderCount;
  lastActualEncoder = EncoderCount;
}
void setDesiredEncoderSpeed(double increment){
  incremental = increment;
}
double getKpEncoder() {
  return Kp_Encoder;
}
double getKiEncoder() {
  return Ki_Encoder;
}
double getKdEncoder() {
  return Kd_Encoder;
}
double getDesiredEncoder(){
  return desiredEncoder;
}
double getLastEncoder(){
  return lastActualEncoder;
}
// Calculates the PID output
double ComputeEncoderPID(long encoderCounts )
{
  if(desiredEncoder < userDesiredEncoder){
    desiredEncoder+=incremental;
  }else{
    desiredEncoder-=incremental;
  }
  double actualEncoder = encoderCounts / 10.0;
  static double integralError = 0, output;
  double error = desiredEncoder - actualEncoder;
  //derivative Ki_Encoderck
  long dactualEncoder = (lastActualEncoder - actualEncoder);

  // Handle Integral Windup with Clamping
  if (output < MAX_ANGLE_FORWARD && output > -MAX_ANGLE_BACKWARD) {
    integralError += error;
  }

  // Compute PID Output
  output = Kp_Encoder * error + Ki_Encoder * integralError + Kd_Encoder * dactualEncoder;

  //Saturation
  if (output > MAX_ANGLE_FORWARD) output = MAX_ANGLE_FORWARD;
  else if (output < -(MAX_ANGLE_BACKWARD)) output = -(MAX_ANGLE_BACKWARD);

  lastActualEncoder = actualEncoder;
  return output;
}

void printEncoderInfo() {
  Serial.print("Position Info: ");
  Serial.print("Kp=");
  Serial.print(Kp_Encoder,4);
  Serial.print(" , ");
  Serial.print("Ki=");
  Serial.print(Ki_Encoder,4);
  Serial.print(" , ");
  Serial.print("Kd=");
  Serial.print(Kd_Encoder,4);
  Serial.print(" , ");
  Serial.print("desired=");
  Serial.print(desiredEncoder);
  Serial.print(" , ");
  Serial.print("Actual=");
  Serial.print(lastActualEncoder);
  Serial.print("\n");
}
