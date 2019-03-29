#include <Arduino.h>

#include <math.h>
#include <string>

#include "Source/MotorDriver.h"
#include "Source/Orientation.h"

#include <EasyTransfer.h>
//#include <i2c_t3.h>
#include <Wire.h>

//#include "MPU6050_6Axis_MotionApps20.h"
struct SEND_DATA_STRUCTURE{
    float angle;
    bool stab;
};

EasyTransfer ET;
elapsedMillis stabTimer;

SEND_DATA_STRUCTURE mydata;

void setup() {
    Serial.begin(115200);
    Serial.println("test");
    Orientation::init();

    mydata.stab = false;
    ET.begin(details(mydata), &Serial);
}

void loop() {
    Orientation::update();

    mydata.angle = Orientation::getYaw();
    //mydata.stab = Orientation::isStabalized();

    //Serial.println(Orientation::getYaw());
    // if (mydata.stab || stabTimer > 100) {
    //     ET.sendData();
    //     // debugLightSensors();
    //     stabTimer = 0;
    // }
}
