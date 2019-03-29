#include <Arduino.h>

#include "Source/Orientation.h"

#include <EasyTransfer.h>
#include <elapsedMillis.h>

struct SEND_DATA_STRUCTURE{
    float angle;
    bool stab;
};

EasyTransfer ET;
elapsedMillis stabTimer;

SEND_DATA_STRUCTURE mydata;

void setup() {
    Serial.begin(115200);
    Orientation::init();

    mydata.stab = false;
    ET.begin(details(mydata), &Serial);
}

void loop() {
    Orientation::update();

    mydata.angle = Orientation::getYaw();
    mydata.stab = Orientation::isStabalized();
    Serial.println(Orientation::getYaw());
    if (mydata.stab || stabTimer > 100 && false) {
        ET.sendData();
        // debugLightSensors();
        stabTimer = 0;
    }
}
