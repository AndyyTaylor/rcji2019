#ifndef Teensy_MotorCarrier_h
#define Teensy_MotorCarrier_h

#include <Arduino.h>

class Teensy_MotorCarrier
{
  public:
    // CONSTRUCTORS
    //Teensy_MotorCarrier(); // Default pin selection.
    Teensy_MotorCarrier(unsigned char M1IN1, unsigned char M1IN2M, unsigned char M1PWM,
                           unsigned char M2IN1, unsigned char M2IN2M, unsigned char M2PWM); // User-defined pin selection.

    // PUBLIC METHODS
    void init(); // Initialize TIMER 1, set the PWM to 20kHZ.
    void setM1Speed(int speed); // Set speed for M1.
    void setM2Speed(int speed); // Set speed for M2.
    void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
    unsigned int getM1CurrentMilliamps(); // Get current reading for M1.
    unsigned int getM2CurrentMilliamps(); // Get current reading for M2.
    unsigned char getFault(); // Get fault reading.

  private:
    unsigned char _M1IN1;
    unsigned char _M1IN2;
    unsigned char _M1PWM;
    unsigned char _M2IN1;
    unsigned char _M2IN2;
    unsigned char _M2PWM;
};

#endif
