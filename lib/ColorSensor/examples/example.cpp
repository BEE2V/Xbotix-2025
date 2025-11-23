#include <Arduino.h>
#include "ColorSensor.h"

ColorSensor sensor(3, A0);

void setup()
{
    Serial.begin(115200);
    sensor.begin();
    sensor.setAverageSamples(5);
    sensor.calibrate();
}

void loop()
{
    int r, g, b;
    sensor.readRGB(r, g, b);

    Serial.print("RGB = ");
    Serial.print(r);
    Serial.print(" ");
    Serial.print(g);
    Serial.print(" ");
    Serial.println(b);

    int id = sensor.detectColor();
    Serial.print("Detected: ");
    Serial.println(sensor.colorName(id));

    delay(300);
}
