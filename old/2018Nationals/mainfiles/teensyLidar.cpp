#include <Arduino.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_HMC5883_U.h>
#include <EasyTransfer.h>
#include "I2Cdev.h"
#include "Wire.h"
#include "elapsedMillis.h"
#include "RPLidar.h"

// You need to create an driver instance
RPLidar lidar;

#define RPLIDAR_MOTOR 2

struct TEENSY_MAIN_OUT {
    int16_t xValue;
    int16_t yValue;
    int16_t xAccuracy;
    int16_t yAccuracy;
};

struct TEENSY_MAIN_IN {
    int16_t gyro;
    int16_t button;
};

EasyTransfer ETin, ETout;
elapsedMillis checkTimer;
elapsedMillis sendTimer;

elapsedMillis outTimer;

elapsedMillis serialTimer;

TEENSY_MAIN_IN receiveData;
TEENSY_MAIN_OUT sendData;

int counter = 0;

float minX = 0;
float maxX = 0;
float minY = 0;
float maxY = 0;

float xVal;
float yVal;

float accuracyX;
float accuracyY;

float fieldWidth = 1820;
float fieldHeight = 2430;


double roboAngle = 0;
void loop();
void debug();
void updateLidar();

bool lidarON = false;
int scanCount = 0;

bool DEBUG = false;


void setup() {
    Serial.begin(115200);
    pinMode(13,OUTPUT);

    Serial4.begin(115200);
    ETin.begin(details(receiveData), &Serial4);
    ETout.begin(details(sendData), &Serial4);

    lidar.begin(Serial2);
    pinMode(RPLIDAR_MOTOR, OUTPUT);
}

void loop() {
    if (lidarON) {
      updateLidar();
    }
    else {
      analogWrite(RPLIDAR_MOTOR, 0);
      xVal = fieldWidth/2;
      yVal = fieldHeight/2;
      accuracyX = fieldWidth/2;
      accuracyY = fieldHeight/2;
    }

    if (sendTimer > 10) {
      sendData.xValue = xVal;
      sendData.yValue = yVal;
      sendData.xAccuracy = accuracyX;
      sendData.yAccuracy = accuracyY;
      ETout.sendData();

      sendTimer = 0;
    }

    if (ETin.receiveData()) {
      roboAngle = receiveData.gyro / 180.0f * PI;
      lidarON = receiveData.button;
    }

    //debug();
}

void updateLidar() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

    if (angle > 180) {angle -= 360;}

    if (startBit) { scanCount++;}

    if (scanCount >= 1) {
      counter ++;
      if (DEBUG) {
        Serial.println("--------");
        Serial.print("(");
        Serial.print(minX);
        Serial.print(", ");
        Serial.print(maxX);
        Serial.print(") (");
        Serial.print(minY);
        Serial.print(", ");
        Serial.print(maxY);
        Serial.println(")");
        Serial.println("--------");
      }
      xVal = (-minX + fieldWidth - maxX)/2;
      yVal = (-minY + fieldHeight - maxY)/2;

      accuracyX = abs(fieldWidth - (abs(minX)+abs(maxX)))/2;
      accuracyY = abs(fieldHeight - (abs(minY)+abs(maxY)))/2;


      minX = 0;
      maxX = 0;
      minY = 0;
      maxY = 0;

      scanCount = 0;
    }

    if (distance != 0) {
//      Serial.print(distance);
//      Serial.print(", ");
//      Serial.println(angle);


      float tempX = distance*sin(roboAngle-angle*3.14/180);
      float tempY = distance*cos(roboAngle-angle*3.14/180);
    if (DEBUG) {
     Serial.print(tempX);
     Serial.print(", ");
     Serial.println(tempY);
    }
     //Serial.println();

      if (tempX > maxX) { maxX = tempX;}
      if (tempY > maxY) { maxY = tempY;}
      if (tempX < minX) { minX = tempX;}
      if (tempY < minY) { minY = tempY;}
    }



  } else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor

    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       lidar.startScan();

       // start motor rotating at max allowed speed
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    }
  }
}



void debug() {
  if (serialTimer > 100) {
  Serial.print("ON: ");
    Serial.print(lidarON);

    Serial.print(" Position: ");
    Serial.print(xVal);
    Serial.print(" (");
    Serial.print(accuracyX);
    Serial.print("), ");
    Serial.print(yVal);
    Serial.print(" (");
    Serial.print(accuracyY);
    Serial.print("), ");

    Serial.print(" Gyro: ");
    Serial.print(roboAngle);

    Serial.println();
    serialTimer = 0;
  }
}
