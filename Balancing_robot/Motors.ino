// Motor Pins
#define LEFT_MOTOR_APWM   10
#define LEFT_MOTOR_AIN1   6
#define LEFT_MOTOR_AIN2   5

#define RIGHT_MOTOR_BIN1  7
#define RIGHT_MOTOR_BIN2  8
#define RIGHT_MOTOR_BPWM  9

//#define TOP_ICR 0x03FF

// Prueba set point
double distancia = 0;
int resolution = 1028;
void InitMotors(){
    // Motor
  pinMode(RIGHT_MOTOR_BPWM, OUTPUT);  
  pinMode(RIGHT_MOTOR_BIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_BIN2, OUTPUT); 
  
  pinMode(LEFT_MOTOR_APWM, OUTPUT);  
  pinMode(LEFT_MOTOR_AIN1, OUTPUT);  
  pinMode(LEFT_MOTOR_AIN2, OUTPUT); 
 
  digitalWrite(RIGHT_MOTOR_BIN1,HIGH); 
  digitalWrite(RIGHT_MOTOR_BIN2,HIGH); 
  digitalWrite(LEFT_MOTOR_AIN1,HIGH); 
  digitalWrite(LEFT_MOTOR_AIN2,HIGH); 
  
}

void setupPWM16() {
  DDRB  |= _BV(PB1) | _BV(PB2);       /* set pins as outputs */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
           | _BV(WGM11);                 /* mode 14: fast PWM, TOP=ICR1 */
  TCCR1B = _BV(WGM13) | _BV(WGM12)
           | _BV(CS11);                  /* prescaler 8 */
  ICR1 = resolution;                         /* TOP counter value (freeing OCR1A*/
}

/* 16-bit version of analogWrite(). Works only on pins 9 and 10. */
void analogWrite16(uint8_t pin, uint16_t val)
{
  switch (pin) {
    case  9: OCR1A = val; break;
    case 10: OCR1B = val; break;
  }
}

void MotorControl(int out){
  
  // Sets direction
  if (out > 0){
    digitalWrite(RIGHT_MOTOR_BIN1,HIGH);
    digitalWrite(RIGHT_MOTOR_BIN2,LOW);
    digitalWrite(LEFT_MOTOR_AIN1,LOW);
    digitalWrite(LEFT_MOTOR_AIN2,HIGH);
  }else{
    digitalWrite(RIGHT_MOTOR_BIN1,LOW);
    digitalWrite(RIGHT_MOTOR_BIN2,HIGH);
    digitalWrite(LEFT_MOTOR_AIN1,HIGH);
    digitalWrite(LEFT_MOTOR_AIN2,LOW);
  }
  
  int vel = abs(out);
  if (vel<0)
    vel=0;
  if (vel > resolution-50 )
    vel=resolution-50;
  
  //Serial.println(out);
  analogWrite16(RIGHT_MOTOR_BPWM,vel);
  analogWrite16(LEFT_MOTOR_APWM,vel);
  
}