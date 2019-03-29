#include "Teensy_MotorCarrier.h"

// Constructors ////////////////////////////////////////////////////////////////

Teensy_MotorCarrier::Teensy_MotorCarrier(unsigned char M1IN1, unsigned char M1IN2, unsigned char M1PWM,
                                          unsigned char M2IN1, unsigned char M2IN2, unsigned char M2PWM)
{
  //Pin map
  //PWM1 and PWM2 cannot be remapped because the library assumes PWM is on timer1
  _M1IN1 = M1IN1;
  _M1IN2 = M1IN2;
  _M1PWM = M1PWM;
  _M2IN1 = M2IN1;
  _M2IN2 = M2IN2;
  _M2PWM = M2PWM;
}

// Public Methods //////////////////////////////////////////////////////////////
void Teensy_MotorCarrier::init()
{
// Define pinMode for the pins and set the frequency for timer1.
  // Serial.begin(9600);
  // Serial.print(_M1IN1);
  // Serial.print(_M1IN2);
  // Serial.print(_M1PWM);

  pinMode(_M1IN1,OUTPUT);
  pinMode(_M1IN2,OUTPUT);
  pinMode(_M1PWM,OUTPUT);

  pinMode(_M2IN1,OUTPUT);
  pinMode(_M2IN2,OUTPUT);
  pinMode(_M2PWM,OUTPUT);

  analogWriteFrequency(_M1PWM, 20000);
  analogWriteFrequency(_M2PWM, 20000);
  analogWriteResolution(9);
}
// Set speed for motor 1, speed is a number betwenn -400 and 400
void Teensy_MotorCarrier::setM1Speed(int speed)
{
  unsigned char reverse = 0;

  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 511)  // Max PWM dutycycle
    speed = 511;

  analogWrite(_M1PWM,speed);

  if (reverse) {
    digitalWrite(_M1IN1,HIGH);
    digitalWrite(_M1IN2,LOW);
  }
  else {
    digitalWrite(_M1IN1,LOW);
    digitalWrite(_M1IN2,HIGH);
  }
}


void Teensy_MotorCarrier::setM2Speed(int speed)
{
  unsigned char reverse = 0;

  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 511)  // Max PWM dutycycle
    speed = 511;

  analogWrite(_M2PWM,speed); // default to using analogWrite, mapping 400 to 255

  if (reverse) {
    digitalWrite(_M2IN1,HIGH);
    digitalWrite(_M2IN2,LOW);
  }
  else {
    digitalWrite(_M2IN1,LOW);
    digitalWrite(_M2IN2,HIGH);
  }
}

void Teensy_MotorCarrier::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}
