#include "Teensy_MotorShield.h"

// Constructors ////////////////////////////////////////////////////////////////

Teensy_MotorShield::Teensy_MotorShield()
{
  //Pin map
  _nD2 = 4;
  _M1DIR = 7;
  _M2DIR = 8;
  _nSF = 12;
}

Teensy_MotorShield::Teensy_MotorShield(unsigned char nD2, unsigned char M1DIR, unsigned char M2DIR,
                                        unsigned char M1PWM, unsigned char M2PWM, unsigned char nSF)
{
  //Pin map
  //PWM1 and PWM2 cannot be remapped because the library assumes PWM is on timer1
  //(7, 9, A0, 8, 10, A1, 4, 12)
  _nD2 = nD2;
  _M1DIR = M1DIR;
  _M2DIR = M2DIR;
  _nSF = nSF;
  _M1PWM = M1PWM;
  _M2PWM = M2PWM;
}

// Public Methods //////////////////////////////////////////////////////////////
void Teensy_MotorShield::init()
{
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_M1DIR,OUTPUT);
  pinMode(_M1PWM,OUTPUT);
  pinMode(_M2DIR,OUTPUT);
  pinMode(_M2PWM,OUTPUT);
  pinMode(_nD2,OUTPUT);
  digitalWrite(_nD2,HIGH); // default to on
  pinMode(_nSF,INPUT);

  analogWriteFrequency(_M1PWM, 20000);
  analogWriteFrequency(_M2PWM, 20000);
  analogWriteResolution(9);
}
// Set speed for motor 1, speed is a number betwenn -400 and 400
void Teensy_MotorShield::setM1Speed(int speed)
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

  if (reverse)
    digitalWrite(_M1DIR,HIGH);
  else
    digitalWrite(_M1DIR,LOW);
}


void Teensy_MotorShield::setM2Speed(int speed)
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

  if (reverse)
    digitalWrite(_M2DIR,HIGH);
  else
    digitalWrite(_M2DIR,LOW);
}

void Teensy_MotorShield::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

unsigned char Teensy_MotorShield::getFault()
{
  return !digitalRead(_nSF);
}
