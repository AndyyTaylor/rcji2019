#include <Arduino.h>

#include <math.h>
#include <string>

#include "Source/MotorDriver.h"
// #include "Source/Orientation.h"
#include "DualMC33926MotorShield.h"

#include <EasyTransfer.h>
#include <Wire.h>
#include <EasyTransferI2C.h>
#include <elapsedMillis.h>

#include <PixyI2C.h>

struct RECEIVE_DATA_STRUCTURE_GYRO {
  int16_t angle;
  int16_t stab;
  int16_t irValue;
};

elapsedMillis serialTimer;
elapsedMillis outTimer = 10000;
elapsedMillis cameraTimer;

void receive(int numBytes) {}
void receiveGyro();
void cameraAngle();
void setupVars();
void followBall();
void debug();
void outIR();
void lightCheck();

float angleToTarget(int x1, int y1, int x2, int y2);
float distanceToTarget(int x1, int y1, int x2, int y2);

bool initEverything();

void dead();

RECEIVE_DATA_STRUCTURE_GYRO gyroData;

EasyTransfer GyroIn;

bool hitLine = false;
int gyroOffset = 0;

int gyro = 0;
bool stab = false;
float IR_Reading = 0;

bool stop = false;
double desiredDirection = 0;
bool smooth = true;
int off = 0;

int tlMax = 0;
int blMax = 0;
int brMax = 0;
int trMax = 0;

static int lineStop = 2500;

PixyI2C pixy;

double goalAngle = 0;

void loop() {
      cameraAngle();
      receiveGyro();
      if (outTimer > lineStop + 1000) {
        lightCheck();
      }
      if (analogRead(A12) < 850 && analogRead(A14) > 900) {
        digitalWrite(17, HIGH);
        delay(80);
        digitalWrite(17, LOW);
        Serial.println("kick");
      }


      if (digitalRead(A15) == 1) {
        if (cameraTimer < 500) {
          MotorDriver::update(goalAngle);
        }
        else {
          MotorDriver::update(gyro);
        }

        //testIR();
        if (outTimer > lineStop + 1000) {
          followBall();
        }
        else if (outTimer > lineStop) {
          outIR();
        }
        else {
          MotorDriver::stop();
        }
      }
      else {
        MotorDriver::stop();
      }
      debug();
}

void outIR() {
  if (IR_Reading < 1000) {
    desiredDirection = IR_Reading;
    MotorDriver::direction(desiredDirection, true);
  }
  else {
    MotorDriver::stop();
  }
}

void followBall() {
    if (IR_Reading < 1000) {
      if (IR_Reading == 0) { desiredDirection = 0;}
      else if (abs(IR_Reading) <= 54) {
          desiredDirection = IR_Reading + IR_Reading/abs(IR_Reading) * 55;
      }
      else {
          desiredDirection = IR_Reading + IR_Reading/abs(IR_Reading) * 30;
      }
      MotorDriver::direction(desiredDirection);
    }
    else {
      MotorDriver::stop();
    }
}

void lightCheck() {
  if (analogRead(A8) > tlMax) { hitLine = true;}
  else if (analogRead(A9) > blMax) { hitLine = true;}
  else if (analogRead(A10) > brMax) { hitLine = true;}
  else if (analogRead(A11) > trMax) { hitLine = true;}
  else { hitLine = false; }
  if (hitLine) {
    outTimer = 0;
  }
}

void cameraAngle() {
  uint16_t blocks;

  blocks = pixy.getBlocks();

  if (blocks) {
      double angle = 0;
      double largestThing = 0;
      for (int j=0; j<blocks; j++)
      {
        double area = pixy.blocks[j].width * pixy.blocks[j].height;
        if (pixy.blocks[j].signature == 1 && area > largestThing) {
          angle = pixy.blocks[j].x/320.0f*75 - 37.5;
          largestThing = area;
          cameraTimer = 0;
        }
      }
      goalAngle = -angle * 1.5;
  }
  else {
    //goalAngle = gyro;
  }

}

void receiveGyro() {
    if (GyroIn.receiveData()) {

        gyro = gyroData.angle;
        stab = gyroData.stab;
        IR_Reading = gyroData.irValue;
    }
}

void setup() {
    init();
    Serial.begin(115200);
    pinMode(A15, OUTPUT);
    digitalWrite(A15, HIGH);

    pinMode(17, OUTPUT);

    // pinMode(A14, INPUT);
    // pinMode(A12, INPUT);
    // pinMode(A11, INPUT);
    // pinMode(A10, INPUT);
    // pinMode(A9, INPUT);
    // pinMode(A8, INPUT);

    tlMax = analogRead(A8) + 100;
    //Serial.println(tlMax);
    blMax = analogRead(A9) + 100;
    //Serial.println(blMax);
    brMax = analogRead(A10) + 100;
    //Serial.println(brMax);
    trMax = analogRead(A11) + 100;
    //Serial.println(trMax);

    //attachInterrupt(digitalPinToInterrupt(20), lineInterrupt, RISING);

    Serial1.begin(115200);
    Serial.println(F("Initializing"));

    GyroIn.begin(details(gyroData), &Serial1);

    Wire.onReceive(receive);

    pixy.init();

    MotorDriver::init();
    MotorDriver::setMaxSpeed(350);

    return true;
}

void debug() {
    if (serialTimer > 100) {
        Serial.print(digitalRead(A15));
        Serial.print(", ");
        Serial.print(IR_Reading);
        Serial.print(", ");
        Serial.print(stab);
        Serial.print(", ");
        Serial.print(gyro);
        Serial.print(", ");
        Serial.print(desiredDirection);

        Serial.print(" -- Line: ");

        Serial.print(analogRead(A8));
        Serial.print(", ");
        Serial.print(analogRead(A9));
        Serial.print(", ");
        Serial.print(analogRead(A10));
        Serial.print(", ");
        Serial.print(analogRead(A11));
        Serial.print(", ");
        Serial.print(analogRead(A12));
        Serial.print(", ");
        Serial.print(hitLine);

        Serial.print(" -- ");

        Serial.print(analogRead(A14));

        Serial.print(" -- ");

        Serial.print(goalAngle);

        Serial.println();


        serialTimer = 0;
    }
}
