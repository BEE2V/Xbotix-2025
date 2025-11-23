#include "ColorSensor.h"

ColorSensor::ColorSensor(uint8_t ledPin, uint8_t ldrPin)
    : strip(1, ledPin, NEO_GRB + NEO_KHZ800), _ldrPin(ldrPin) {}

void ColorSensor::begin()
{
    strip.begin();
    strip.show();
}

void ColorSensor::countdown(const char *msg)
{
    Serial.println(msg);
    delay(300);
    Serial.println("3");
    delay(400);
    Serial.println("2");
    delay(400);
    Serial.println("1");
    delay(400);
}

int ColorSensor::readOneColor(int r, int g, int b)
{
    strip.setPixelColor(0, strip.Color(r, g, b));
    strip.show();
    delay(30);
    int v = analogRead(_ldrPin);
    strip.setPixelColor(0, 0, 0, 0);
    strip.show();
    return v;
}

int ColorSensor::averagedRead(int r, int g, int b)
{
    long sum = 0;
    for (int i = 0; i < avgSamples; i++)
    {
        sum += readOneColor(r, g, b);
        delay(5);
    }
    return sum / avgSamples;
}

void ColorSensor::readRaw(int &r, int &g, int &b)
{
    r = averagedRead(255, 0, 0);
    g = averagedRead(0, 255, 0);
    b = averagedRead(0, 0, 255);
}

void ColorSensor::readRGB(int &r, int &g, int &b)
{
    int rr, gg, bb;
    readRaw(rr, gg, bb);

    r = map(rr, rLow, rHigh, 0, 255);
    g = map(gg, gLow, gHigh, 0, 255);
    b = map(bb, bLow, bHigh, 0, 255);

    r = constrain(r, 0, 255);
    g = constrain(g, 0, 255);
    b = constrain(b, 0, 255);
}

void ColorSensor::calibrate()
{
    Serial.println("=== CALIBRATION START ===");

    // HIGH = WHITE
    countdown("Show WHITE surface...");
    rHigh = averagedRead(255, 0, 0);
    gHigh = averagedRead(0, 255, 0);
    bHigh = averagedRead(0, 0, 255);

    Serial.println("HIGH:");
    Serial.print("R=");
    Serial.println(rHigh);
    Serial.print("G=");
    Serial.println(gHigh);
    Serial.print("B=");
    Serial.println(bHigh);

    delay(800);

    // LOW = BLACK
    countdown("Show BLACK surface...");
    rLow = averagedRead(255, 0, 0);
    gLow = averagedRead(0, 255, 0);
    bLow = averagedRead(0, 0, 255);

    Serial.println("LOW:");
    Serial.print("R=");
    Serial.println(rLow);
    Serial.print("G=");
    Serial.println(gLow);
    Serial.print("B=");
    Serial.println(bLow);

    Serial.println("=== CALIBRATION DONE ===");
    delay(800);
}

// Detects strongest channel: 0=Red, 1=Green, 2=Blue
int ColorSensor::detectColor()
{
    int r, g, b;
    readRGB(r, g, b);

    if (r > g && r > b)
        return 0;
    if (g > r && g > b)
        return 1;
    return 2; // blue
}

const char *ColorSensor::colorName(int id)
{
    switch (id)
    {
    case 0:
        return "RED";
    case 1:
        return "GREEN";
    case 2:
        return "BLUE";
    default:
        return "UNKNOWN";
    }
}

void ColorSensor::setAverageSamples(uint8_t samples)
{
    avgSamples = samples;
}
