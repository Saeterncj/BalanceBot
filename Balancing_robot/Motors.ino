// Motor Pins
#define LEFT_MOTOR_AIN1   6
#define LEFT_MOTOR_AIN2   5
#define RIGHT_MOTOR_BIN1  7
#define RIGHT_MOTOR_BIN2  8

//#define TOP_ICR 0x03FF

// Prueba set point
double distancia = 0;
int resolution = 1028;
int deadBand = 0;
void initMotors() {
  // Init right motor pins to be ooutputs
  pinMode(RIGHT_MOTOR_BPWM, OUTPUT);
  pinMode(RIGHT_MOTOR_BIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_BIN2, OUTPUT);

  // Init left motor pins to be outputs
  pinMode(LEFT_MOTOR_APWM, OUTPUT);
  pinMode(LEFT_MOTOR_AIN1, OUTPUT);
  pinMode(LEFT_MOTOR_AIN2, OUTPUT);

  digitalWrite(RIGHT_MOTOR_BIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_BIN2, HIGH);
  digitalWrite(LEFT_MOTOR_AIN1, HIGH);
  digitalWrite(LEFT_MOTOR_AIN2, HIGH);
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
void stopMotors() {
  analogWrite16(RIGHT_MOTOR_BPWM, 1);
  analogWrite16(LEFT_MOTOR_APWM, 1);
}
void motorControl(uint8_t whichMotor, int out) {

  // Sets direction
  if (out > 0) {
    if (whichMotor == LEFT_MOTOR_APWM) {
      digitalWrite(LEFT_MOTOR_AIN1, LOW);
      digitalWrite(LEFT_MOTOR_AIN2, HIGH);
      analogWrite16(LEFT_MOTOR_APWM, out);
    } else {
      digitalWrite(RIGHT_MOTOR_BIN1, HIGH);
      digitalWrite(RIGHT_MOTOR_BIN2, LOW);
      analogWrite16(RIGHT_MOTOR_BPWM, out);
    }


  } else {
    out *= -1;
    if (whichMotor == RIGHT_MOTOR_BPWM) {
      digitalWrite(RIGHT_MOTOR_BIN1, LOW);
      digitalWrite(RIGHT_MOTOR_BIN2, HIGH);
      analogWrite16(RIGHT_MOTOR_BPWM, out);
    } else {
      digitalWrite(LEFT_MOTOR_AIN1, HIGH);
      digitalWrite(LEFT_MOTOR_AIN2, LOW);
      analogWrite16(LEFT_MOTOR_APWM, out);
    }
  }
}

//add dt as parameter?
double getLeftMotorSpeed() {
  static long lastLeftEncoderCount = 0;
  long currentLeftEncoderCount = leftEncoders.getEncoderCount();
  long diff = currentLeftEncoderCount - lastLeftEncoderCount;
  lastLeftEncoderCount = currentLeftEncoderCount;
  double actualLeftSpeed = (double)diff * .928;
  return actualLeftSpeed;
}

double getRightMotorSpeed() {
  static long lastRightEncoderCount = 0;
  long currentRightEncoderCount = rightEncoders.getEncoderCount();
  long diff = currentRightEncoderCount - lastRightEncoderCount;
  lastRightEncoderCount = currentRightEncoderCount;
  double actualRightSpeed = (double)diff * .928;
  return actualRightSpeed;
}

void setDB(int db) {
  deadBand = db;
}
int getDB() {
  return deadBand;
}
void findMotorDeadBand() {
  rightEncoders.setEncoderCount(0L);
  leftEncoders.setEncoderCount(0L);
  int pwmVal = 1;
  long start = millis();
  while (leftEncoders.getEncoderCount() == 0 && rightEncoders.getEncoderCount() == 0) {
    motorControl(RIGHT_MOTOR_BPWM, pwmVal);
    motorControl(LEFT_MOTOR_APWM, pwmVal);
    if (millis() - start > 40) {
      start = millis();
      pwmVal++;
    }
  }
  deadBand = pwmVal;
  Serial.print("The value for Deadband is: ");
  Serial.println(deadBand);
}
