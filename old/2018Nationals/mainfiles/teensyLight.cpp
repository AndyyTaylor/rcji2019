#include <Arduino.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_HMC5883_U.h>
#include <EasyTransfer.h>
#include "I2Cdev.h"
#include "Wire.h"
#include "elapsedMillis.h"

struct TEENSY_MAIN_OUT {
    int16_t front;
    int16_t right;
    int16_t back;
    int16_t left;

    int16_t laser;
};

struct TEENSY_MAIN_IN {
    int16_t gyro;
    int16_t button;

    int16_t xValue;
    int16_t yValue;
    int16_t xAccuracy;
    int16_t yAccuracy;
};

EasyTransfer ETin, ETout;
elapsedMillis checkTimer;
elapsedMillis sendTimer;

elapsedMillis outTimer;

elapsedMillis serialTimer;

TEENSY_MAIN_IN receiveData;
TEENSY_MAIN_OUT sendData;

//
// SEND_DATA_STRUCTURE mydata;

double roboAngle = 0;
bool lineTrigger[] = {false,false,false,false};
bool prevTrigger[] = {false,false,false,false};
bool triggerCount = false;
void loop();
void greenSet();
void debug();
void trigger();

bool buttonPressed = false;

bool DEBUG = false;




byte lightSensors[]= {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A11, A10, A12, A13, A14, A15, A16, A17, A18};
int lightValues[19];

float xVal;
float yVal;
float accuracyX;
float accuracyY;

void setup() {
    Serial.begin(115200);
    pinMode(13,OUTPUT);

    Serial1.begin(115200);
    ETin.begin(details(receiveData), &Serial1);
    ETout.begin(details(sendData), &Serial1);

    roboAngle = roboAngle * PI/180;
    greenSet();
}

void loop() {
    debug();
    //Serial.println(analogRead(A4));
    if (outTimer > 750) {
      prevTrigger[0] = false;
      prevTrigger[1] = false;
      prevTrigger[2] = false;
      prevTrigger[3] = false;
    }

    if (sendTimer > 10) {
      sendData.front = lineTrigger[0];
      sendData.right = lineTrigger[1];
      sendData.back = lineTrigger[2];
      sendData.left = lineTrigger[3];
      sendData.laser = analogRead(A19);
      ETout.sendData();
      sendTimer = 0;
    }

    if (ETin.receiveData()) {
      roboAngle = receiveData.gyro / 180.0f * PI;

      xVal = receiveData.xValue;
      yVal = receiveData.yValue;
      accuracyX = receiveData.xAccuracy;
      accuracyY = receiveData.yAccuracy;

      if (receiveData.button) {
        buttonPressed = true;
      }
      else if (buttonPressed) {
        greenSet();
        Serial.println("reset");
        buttonPressed = false;
      }
    }
    trigger();


    //Serial.println();
    //delay(5);
}
void greenSet(){
    for(int(i) = 0; i < 19; i++){
        lightValues[i] = analogRead(lightSensors[i]);
    }
}

void debug() {
  if (serialTimer > 100) {
    Serial.print("Lines: ");
    Serial.print(lineTrigger[0]);
    Serial.print(", ");
    Serial.print(lineTrigger[1]);
    Serial.print(", ");
    Serial.print(lineTrigger[2]);
    Serial.print(", ");
    Serial.print(lineTrigger[3]);

    Serial.print(", GYRO: ");
    Serial.print(roboAngle);

    Serial.print(" Position: ");
    Serial.print(xVal);
    Serial.print(" (");
    Serial.print(accuracyX);
    Serial.print("), ");
    Serial.print(yVal);
    Serial.print(" (");
    Serial.print(accuracyY);
    Serial.print("), ");

    // for(int i = 0; i < 19; i++){
    //   Serial.print(i+1);
    //   Serial.print(":");
    //   Serial.print(analogRead(lightSensors[i]));
    //   Serial.print(" , ");
    // }


    Serial.println();
    serialTimer = 0;
  }
}


void trigger(){
    double xMax = -99;
    double xMin= 99;
    double yMax= -99;
    double yMin = 99;

    double xSum = 0;
    double ySum = 0;

    triggerCount = false;

    for(int(i) = 0; i < 19; i++){

        if(analogRead(lightSensors[i]) > 2*lightValues[i]){
            triggerCount = true;
            if (DEBUG) {
              Serial.print(i + 1);
              Serial.print(", ");
            }
            digitalWrite(13,HIGH);
            double xPos;
            double yPos;

            if (i < 12) {
              xPos = sin((i) * -PI/6 + roboAngle) * 63.5;
              yPos = cos((i) * -PI/6 + roboAngle) * 63.5;
            }
            else if (i < 14) {
              xPos = sin((i-10) * -PI/6 + roboAngle) * 82.5;
              yPos = cos((i-10) * -PI/6 + roboAngle) * 82.5;
            }
            else if (i < 17) {
              xPos = sin((i-9) * -PI/6 + roboAngle) * 82.5;
              yPos = cos((i-9) * -PI/6 + roboAngle) * 82.5;
            }
            else {
              xPos = sin((i-8) * -PI/6 + roboAngle) * 82.5;
              yPos = cos((i-8) * -PI/6 + roboAngle) * 82.5;
            }

            // Serial.print(i+1);
            // Serial.print(": ");
            // Serial.print(xPos);
            // Serial.print(", ");
            // Serial.print(yPos);
            // Serial.print(" ");


            xSum += xPos;
            ySum += yPos;
            xMax = max(xPos,xMax);
            xMin = min(xPos,xMin);
            yMax = max(yPos,yMax);
            yMin = min(yPos,yMin);


            if(abs(xMax) != 99){
                outTimer = 0;
                // Serial.print(", ");
                // Serial.println(abs(yMax - yMin));
                if(abs(xMax - xMin) > 55){
                    if (accuracyY < 600 && yVal > 1530) {
                      lineTrigger[0] = true;
                      prevTrigger[0] = true;
                      lineTrigger[2] = false;
                      prevTrigger[2] = false;
                    }
                    else if (accuracyY < 600 && yVal < 900) {
                      lineTrigger[2] = true;
                      prevTrigger[2] = true;
                      lineTrigger[0] = false;
                      prevTrigger[0] = false;
                    }
                    else if (prevTrigger[0]) {
                      lineTrigger[0] = true;
                    }
                    else if (prevTrigger[2]) {
                      lineTrigger[2] = true;
                    }
                    else if(ySum > 0 && lineTrigger[2] == false){
                        lineTrigger[0] = true;
                        prevTrigger[0] = true;
                    }
                    else if(ySum < 0 && lineTrigger[0] == false){
                        lineTrigger[2] = true;
                        prevTrigger[2] = true;
                    }
                }


                if(abs(yMax - yMin) > 55){
                    if (accuracyX < 450 && xVal > 1080) {
                      lineTrigger[1] = true;
                      prevTrigger[1] = true;
                      lineTrigger[3] = false;
                      prevTrigger[3] = false;
                    }
                    else if (accuracyX < 450 && xVal < 750) {
                      lineTrigger[3] = true;
                      prevTrigger[3] = true;
                      lineTrigger[1] = false;
                      prevTrigger[1] = false;
                    }
                    else if (prevTrigger[3]) {
                      lineTrigger[3] = true;
                    }
                    else if (prevTrigger[1]) {
                      lineTrigger[1] = true;
                    }
                    else if(xSum > 0 && lineTrigger[3] == false){
                        lineTrigger[1] = true;
                        prevTrigger[1] = true;
                    }
                    else if(xSum < 0 && lineTrigger[1] == false){
                        lineTrigger[3] = true;
                        prevTrigger[3] = true;
                    }
                }
            }
        }
    }

    if(triggerCount == false){
        lineTrigger[1] = false;
        lineTrigger[3] = false;
        lineTrigger[0] = false;
        lineTrigger[2] = false;
        digitalWrite(13,LOW);
    }
    if (DEBUG) {
      Serial.println();
    }
}
