#ifndef Teensy_MotorShield_h
#define Teensy_MotorShield_h

#include <Arduino.h>

class Teensy_MotorShield
{
  public:
    // CONSTRUCTORS
    Teensy_MotorShield(); // Default pin selection.
    Teensy_MotorShield(unsigned char M1DIR, unsigned char M1PWM,
                           unsigned char M2DIR, unsigned char M2PWM,
                           unsigned char nD2, unsigned char nSF); // User-defined pin selection.

    // PUBLIC METHODS
    void init(); // Initialize TIMER 1, set the PWM to 20kHZ.
    void setM1Speed(int speed); // Set speed for M1.
    void setM2Speed(int speed); // Set speed for M2.
    void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
    unsigned int getM1CurrentMilliamps(); // Get current reading for M1.
    unsigned int getM2CurrentMilliamps(); // Get current reading for M2.
    unsigned char getFault(); // Get fault reading.

  private:
    unsigned char _nD2;
    unsigned char _M1DIR;
    unsigned char _M2DIR;
    unsigned char _M1PWM;
    unsigned char _M2PWM;
    unsigned char _nSF;
};

#endif
