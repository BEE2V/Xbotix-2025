#pragma once
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

class ColorSensor
{
public:
    ColorSensor(uint8_t ledPin, uint8_t ldrPin);

    void begin();
    void calibrate();

    void readRaw(int &r, int &g, int &b);
    void readRGB(int &r, int &g, int &b);

    int detectColor(); // returns enum number
    const char *colorName(int id);

    void setAverageSamples(uint8_t samples);

private:
    Adafruit_NeoPixel strip;
    uint8_t _ldrPin;

    int rHigh, gHigh, bHigh;
    int rLow, gLow, bLow;

    uint8_t avgSamples = 5;

    int readOneColor(int r, int g, int b);
    int averagedRead(int r, int g, int b);
    void countdown(const char *msg);
};
