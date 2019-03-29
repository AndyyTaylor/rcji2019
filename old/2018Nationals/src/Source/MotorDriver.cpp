#include "MotorDriver.h"

#include <Arduino.h>
#include <elapsedMillis.h>

#include <Teensy_MotorCarrier.h>

namespace MotorDriver {
    //(7, 9, A0, 8, 10, A1, 4, 12)
    Teensy_MotorCarrier bt(24, 25, 29, 26, 27, 30);
    Teensy_MotorCarrier tp(18, 19, 20, 21, 22, 23);

    elapsedMillis speedTimer;

    int fieldHeight = 2420;
    int fieldWidth = 1830;
    int lineDistance = 300;

    float xVal = 0;
    float yVal = 0;

    double motorSpeed = 200;
    int maxSpeed = 300;
    int curSpeed = 0;
    int prevSpeed = 0;
    double curOrientation = 0;

    bool triggeredLines[] = {false, false, false, false};

    double curAngle = 0;
    double lastChange = 0;

    double offset = 0;

    bool rememberAim = false;

    void update(double orientation, bool lineTest[4], double goalPos, float xval, float yval) {
        curOrientation = goalPos;

        xVal = xval;
        yVal = yval;

        for (int i = 0; i < 4; i ++) {
          triggeredLines[i] = lineTest[i];
        }

        offset = orientation / 180.0f * PI;
    }

    void init(int fwidth, int fheight, int ldist) {
        fieldWidth = fwidth;
        fieldHeight = fheight;
        lineDistance = ldist;

        tp.init();
        bt.init();
    }

    void setSpeed(int speed) {
        motorSpeed = speed;
    }

    void stop() {
        int speed = 0;

        bt.setM1Speed(speed);
        bt.setM2Speed(speed);
        tp.setM1Speed(-speed);
        tp.setM2Speed(-speed);
    }

    void direction(double inangle) {
        curAngle = inangle;

        inangle = curAngle/180.0f*PI + offset;

        double vectorX = motorSpeed*sin(inangle);
        double vectorY = motorSpeed*cos(inangle);

        const int inHeading = 50;

        if (triggeredLines[0] && vectorY > -inHeading) { vectorY = -inHeading;}
        if (triggeredLines[1] && vectorX > -inHeading) { vectorX = -inHeading;}
        if (triggeredLines[2] && vectorY < inHeading) { vectorY = inHeading;}
        if (triggeredLines[3] && vectorX < inHeading) { vectorX = inHeading;}

        //THIS WAS NOT DONE FOR NATIONALS
        // vectorX = min(max(min(((fieldWidth-lineDistance) - xVal)*1.5, 350), -350), vectorX);
        // vectorX = max(max(min(((lineDistance) - xVal)*1.5, 350), -350), vectorX);
        // vectorY = min(max(min(((fieldHeight-lineDistance) - yVal)*1.5, 350), -350), vectorY);
        // vectorY = max(max(min(((lineDistance) - yVal)*1.5, 350), -350), vectorY);
        //Serial.println(fieldHeight);
        //Serial.println(testVector);


        inangle = atan2(vectorX, vectorY) - offset;
        double correctMotorSpeed = sqrt(vectorX*vectorX + vectorY*vectorY);

        // if (triggeredLines[0] || triggeredLines[1] || triggeredLines[2] || triggeredLines[3]) {
        //   correctMotorSpeed = 0;
        // }

        double m2 = cos(inangle+PI/4) * correctMotorSpeed;
        double m1 = cos(inangle-PI/4) * correctMotorSpeed;

        bt.setM1Speed(correct(-m1));
        bt.setM2Speed(correct(-m2));
        tp.setM1Speed(correct(m1));
        tp.setM2Speed(correct(m2));
    }

    int getmotorSpeed() {
        /*Serial.print(speedTimer);
        Serial.print(" : ");
        Serial.print(curSpeed);
        Serial.print(" -> ");
        Serial.print(motorSpeed);
        Serial.print(" (");
        Serial.print(motorSpeed-prevSpeed);
        Serial.println(")");*/
        return motorSpeed;
    }

    double correct(double speed) {
      double newOrientation = curOrientation;
      if (curOrientation > 180) {newOrientation -=360;}
      return max(-400, min(400, speed - newOrientation*2));
    }

    double relativeAngle(double input) {
        while (input > 180) {input -= 360;}
        while (input < -180) {input +=360;}
        return input;
    }
}   // namespace MotorDriver
