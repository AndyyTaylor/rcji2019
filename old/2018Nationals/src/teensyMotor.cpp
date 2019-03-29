#include <Arduino.h>

#include <math.h>
#include <string>

#include "Source/MotorDriver.h"
// #include "Source/Orientation.h"

#include <EasyTransfer.h>
#include <Wire.h>
#include <Pixy2UART.h>
#include <EasyTransferI2C.h>
#include <elapsedMillis.h>

struct GYRO_IN {
  int16_t angle;
  int16_t stab;
};

struct GYRO_OUT {
  int16_t button;
};

struct LIDAR_IN {
    int16_t xValue;
    int16_t yValue;
    int16_t xAccuracy;
    int16_t yAccuracy;
};

struct LIDAR_OUT {
    int16_t gyro;
    int16_t button;
};

struct LIGHT_IN {
    int16_t front;
    int16_t right;
    int16_t back;
    int16_t left;

    int16_t laser;
};

struct LIGHT_OUT {
    int16_t gyro;
    int16_t button;

    int16_t xValue;
    int16_t yValue;
    int16_t xAccuracy;
    int16_t yAccuracy;
};

elapsedMillis serialTimer;
elapsedMillis outTimer = 10000;
elapsedMillis cameraTimer;
elapsedMillis goalTimer;

elapsedMillis sendTimer;
elapsedMillis pixyTimer;

Pixy2UART pixy;

void receiveGyro();
void receiveLidar();
void receiveLight();
void cameraAngle();
void setupVars();
void followBall();
void debug();
void outIR();
void lightCheck();
void positionFront(bool forward);

void followBall2();
void followBallSet();

void goToPoint(int x1, int y1);

float angleToTarget(int x1, int y1, int x2, int y2);
float distanceToTarget(int x1, int y1, int x2, int y2);

bool initEverything();

void dead();

GYRO_IN gyroIn;
GYRO_OUT gyroOut;

LIDAR_IN lidarIn;
LIDAR_OUT lidarOut;

LIGHT_IN lightIn;
LIGHT_OUT lightOut;

EasyTransfer LightIn, LightOut, GyroIn, GyroOut, LidarIn, LidarOut;

bool hitLine = false;
int gyroOffset = 0;

int gyro = 0;
int frontLaser;
bool stab = false;
float IR_Reading = 0;
int frontIR = 0;
bool button2 = false;

bool stop = false;
double desiredDirection = 0;
bool smooth = true;
int off = 0;

double aim = 0;

bool lineTrigger[] = {false, false, false, false};

double goalAngle = 0;
double ballAngle = 0;
double ballDistance = 0;

float xVal;
float yVal;
float accuracyX;
float accuracyY;

int fieldHeight = 2420;
int fieldWidth = 1830;
int lineDistance = 400;

bool saveRotation = false;
float savedGyro = 0;

void loop() {
  receiveGyro();
  receiveLidar();
  receiveLight();

  if (pixyTimer > 30) {
      cameraAngle();
      pixyTimer = 0;
    }

  if (sendTimer > 10) {
    gyroOut.button = digitalRead(6);
    GyroOut.sendData();

    lidarOut.button = digitalRead(5);
    lidarOut.gyro = gyro;
    LidarOut.sendData();

    lightOut.button = digitalRead(4);
    lightOut.gyro = gyro;
    lightOut.xValue = xVal;
    lightOut.yValue = yVal;
    lightOut.xAccuracy = accuracyX;
    lightOut.yAccuracy = accuracyY;
    LightOut.sendData();

    sendTimer = 0;
  }

  if (analogRead(A15) > 900 && frontLaser < 75 && abs(aim) < 15) {
    digitalWrite(A14, HIGH);
    delay(80); //Experiment with this value to optimise the kick
    digitalWrite(A14, LOW);
    delay(2);
  }

if (lineTrigger[0] || lineTrigger[1] || lineTrigger[2] || lineTrigger[3]) {
    saveRotation = true;
}
else {
    saveRotation = false;
}
  if (saveRotation) {
      aim = gyro - savedGyro;
  }
  else if (goalTimer < 500) {
    aim = -goalAngle*1.3;
  }
  else {
    float t_xVal = xVal;
    float t_yVal = yVal;
    if (accuracyX > 450) { t_xVal = 910;}
    if (accuracyY > 600) { t_yVal = 1215;}

    aim = gyro - angleToTarget(t_xVal, t_yVal, 910, 2130);
    savedGyro = gyro;
    }

  if (digitalRead(3)) {
    MotorDriver::update(gyro, lineTrigger, gyro, xVal, yVal);
    MotorDriver::setSpeed(350);
    MotorDriver::direction(180);

    //MotorDriver::update(gyro, lineTrigger, aim, xVal, yVal);
    //followBall2();
  }
  else {
    MotorDriver::stop();
  }
  debug();
}

void followBall() {
    if (ballAngle < 999) {
        MotorDriver::setSpeed(200);
      if (abs(ballAngle) <= 10) {
        //positionFront(true);
        MotorDriver::direction(0);
      }
      else {
          desiredDirection = ballAngle + ballAngle/abs(ballAngle) * 55;
          MotorDriver::direction(desiredDirection);
      }
    }
    else {
      MotorDriver::stop();
    }
}

void goToPoint(int x1, int y1) {
    int driveAngle = angleToTarget(xVal, yVal, x1, y1);
    int driveDistance = distanceToTarget(xVal, yVal, x1, y1);
    if (driveDistance > 200 && digitalRead(5)) {
      if (driveDistance > 750) { MotorDriver::setSpeed(300);}
      else if (driveDistance > 400) { MotorDriver::setSpeed(250);}
      else { MotorDriver::setSpeed(200);}
      MotorDriver::direction(driveAngle-gyro);
    }
    else {
      MotorDriver::stop();
    }

}

void followBall2() {
  if (ballAngle < 999) {
    // if (ballDistance > 50) {
    //   desiredDirection = ballAngle + ballAngle/abs(ballAngle) * 35;
    //   MotorDriver::setSpeed(300);
    //   MotorDriver::direction(desiredDirection);
    // }
    // else if (abs(ballAngle > 45)) {
    //   desiredDirection = ballAngle + ballAngle/abs(ballAngle) * 55;
    //   MotorDriver::setSpeed(200);
    //   MotorDriver::direction(desiredDirection);
    // }
    // else if (abs(ballAngle) < 15) {
    //   MotorDriver::direction(0);
    //   MotorDriver::setSpeed(300);
    // }
    // else {
    //   desiredDirection = ballAngle * 2.3;
    //   MotorDriver::setSpeed(200);
    //   MotorDriver::direction(desiredDirection);
    // }
    int sign = ballAngle/abs(ballAngle);
    if (ballDistance < 65) {
        if (abs(ballAngle) < 20) {
          MotorDriver::direction(0);
          MotorDriver::setSpeed(300);
        }
        else if (abs(ballAngle) < 30) {
        MotorDriver::direction(sign * 75);
        MotorDriver::setSpeed(300);
        }
        else if (abs(ballAngle) < 40) {
        MotorDriver::direction(sign * 95);
        MotorDriver::setSpeed(200);
        }
        else if (abs(ballAngle) < 60) {
        MotorDriver::direction(sign * 110);
        MotorDriver::setSpeed(200);
        }
        else if (abs(ballAngle) < 90) {
        MotorDriver::direction(sign * 145);
        MotorDriver::setSpeed(200);
        }
        else if (abs(ballAngle) < 120) {
        MotorDriver::direction(sign * 200);
        MotorDriver::setSpeed(250);
        }
        else if (abs(ballAngle) < 181) {
        MotorDriver::direction(sign * 255);
        MotorDriver::setSpeed(250);
        }
    }
    else {
        if (abs(ballAngle) < 15) {
          MotorDriver::direction(0);
          MotorDriver::setSpeed(300);
        }
        else if (abs(ballAngle) < 25) {
        MotorDriver::direction(sign * 50);
        MotorDriver::setSpeed(300);
        }
        else if (abs(ballAngle) < 40) {
        MotorDriver::direction(sign * 65);
        MotorDriver::setSpeed(300);
        }
        else if (abs(ballAngle) < 60) {
        MotorDriver::direction(sign * 85);
        MotorDriver::setSpeed(300);
        }
        else if (abs(ballAngle) < 90) {
        MotorDriver::direction(sign * 115);
        MotorDriver::setSpeed(250);
        }
        else if (abs(ballAngle) < 120) {
        MotorDriver::direction(sign * 165);
        MotorDriver::setSpeed(300);
        }
        else if (abs(ballAngle) < 181) {
        MotorDriver::direction(sign * 210);
        MotorDriver::setSpeed(300);
        }
    }
}
else {
    goToPoint(910, 700);
  }
}

void cameraAngle() {
  uint16_t blocks;
  if (cameraTimer > 500) {
    ballAngle = 999;
  }

  //Serial.print("gotHere");
  blocks = pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks) {
  //Serial.println(pixy.ccc.numBlocks);
      double angle = 0;
      bool check = true;
      bool check2 = true;
      //double largestThing = 0;
      for (int j=0; j<blocks; j++)
      {
        //double area = pixy.ccc.blocks[j].width * pixy.ccc.blocks[j].height;
        if (pixy.ccc.blocks[j].m_signature == 1 && check) {
          double ballX = pixy.ccc.blocks[j].m_x - 316.0f/2.0f-10;
          double ballY = pixy.ccc.blocks[j].m_y - 208.0f/2.0f-10;

          cameraTimer = 0;
          ballAngle = atan2(-ballX, -ballY)/3.14f*180;
          ballDistance = sqrt(ballX*ballX+ballY*ballY);
          // Serial.print(ballX);
          // Serial.print(", ");
          // Serial.println(ballY);

          check = false;

        }

        if (pixy.ccc.blocks[j].m_signature == 2 && check2) {
          double goalX = pixy.ccc.blocks[j].m_x - 316.0f/2.0f-10;
          double goalY = pixy.ccc.blocks[j].m_y - 208.0f/2.0f-10;

          goalTimer = 0;
          goalAngle = atan2(-goalX, -goalY)/3.14f*180;
          //ballDistance = sqrt(ballX*ballX+ballY*ballY);
          // Serial.print(ballX);
          // Serial.print(", ");
          // Serial.println(ballY);

          check2 = false;

        }
      }
  }
}

void receiveGyro() {
    if (GyroIn.receiveData()) {
        gyro = gyroIn.angle;
        stab = gyroIn.stab;
    }
}

void receiveLidar() {
    if (LidarIn.receiveData()) {
        xVal = lidarIn.xValue;
        yVal = lidarIn.yValue;
        accuracyX = lidarIn.xAccuracy;
        accuracyY = lidarIn.yAccuracy;
    }
}

void receiveLight() {
    if (LightIn.receiveData()) {
        lineTrigger[0] = lightIn.front;
        lineTrigger[1] = lightIn.right;
        lineTrigger[2] = lightIn.back;
        lineTrigger[3] = lightIn.left;

        frontLaser = lightIn.laser;
    }
}

void setup() {
    pixy.init();

    //BUTTON SETUP
    pinMode(3, OUTPUT);
    digitalWrite(3, HIGH);
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH);
    pinMode(6, OUTPUT);
    digitalWrite(6, HIGH);

    pinMode(A14, OUTPUT);

    Serial.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);
    Serial4.begin(115200);
    Serial.println(F("Initializing"));

    GyroIn.begin(details(gyroIn), &Serial4);
    GyroOut.begin(details(gyroOut), &Serial4);

    LidarIn.begin(details(lidarIn), &Serial2);
    LidarOut.begin(details(lidarOut), &Serial2);

    LightOut.begin(details(lightOut), &Serial3);
    LightIn.begin(details(lightIn), &Serial3);

    MotorDriver::init(fieldWidth, fieldHeight, lineDistance);
    //MotorDriver::setSpeed(400);
}

float angleToTarget(int x1, int y1, int x2, int y2) {
  float cX = x2 - x1;
  float cY = y2 - y1;

  // Serial.print(x2);
  // Serial.print(", ");
  // Serial.println(x1);
  // delay(10);

  return atan2(cX, cY)*180/3.14;
}

float distanceToTarget(int x1, int y1, int x2, int y2) {
  float cX = x2 - x1;
  float cY = y2 - y1;

  return sqrt(cX*cX + cY*cY);
}

void debug() {
    if (serialTimer > 100) {
        Serial.println();


        Serial.print("BALL: ");
        Serial.print(ballAngle);
        Serial.print(", Distance: ");
        Serial.print(ballDistance);

        Serial.print(", GYRO: ");
        Serial.print(gyro);
        // Serial.print(", FRONT_IR: ");
        // Serial.print(frontIR);

        Serial.print(", LINES: ");
        Serial.print(lineTrigger[0]);
        Serial.print(", ");
        Serial.print(lineTrigger[1]);
        Serial.print(", ");
        Serial.print(lineTrigger[2]);
        Serial.print(", ");
        Serial.print(lineTrigger[3]);
        Serial.print(", ");

        Serial.print(" Position: ");
        Serial.print(xVal);
        Serial.print(" (");
        Serial.print(accuracyX);
        Serial.print("), ");
        Serial.print(yVal);
        Serial.print(" (");
        Serial.print(accuracyY);
        Serial.println(")");


        Serial.print("Buttons: ");
        Serial.print(digitalRead(3));
        Serial.print(", ");
        Serial.print(digitalRead(4));
        Serial.print(", ");
        Serial.print(digitalRead(5));
        Serial.print(", ");
        Serial.print(digitalRead(6));
        Serial.print(", ");

        Serial.print("GoalANGLE: ");
        Serial.print(aim);

        Serial.print(", FrontLaser: ");
        Serial.print(frontLaser);

        Serial.print(", KickerCharge: ");
        Serial.print(analogRead(A15));


        Serial.println();
        serialTimer = 0;
    }
}

// void oldLoop() {
//   if (goalTimer < 500 && false) {
//     aim = goalAngle;
//   }
//   else {
//     aim = gyro;
//   }
//
//
//   if (analogRead(A18) < 200 && analogRead(A15) > 750 && abs(aim) < 15) {
//     digitalWrite(A14, HIGH);
//     delay(70);
//     digitalWrite(A14, LOW);
//   }
//
//   if (pixyTimer > 10) {
//     cameraAngle();
//     pixyTimer = 0;
//   }
//   receiveGyro();
//   receiveLight();
//
//   if (sendTimer > 20) {
//     lightOUT.gyro = gyro;
//     lightOUT.button = button2;
//     ETout.sendData();
//     sendTimer = 0;
//   }
//
//   if (digitalRead(29) == 1) {
//
//     //Serial.println(aim);
//     MotorDriver::update(gyro, lineTrigger, aim);
//     followBall();
//     //MotorDriver::direction(90);
//   }
//   else {
//     MotorDriver::stop(false);
//   }
//   debug();
// }
