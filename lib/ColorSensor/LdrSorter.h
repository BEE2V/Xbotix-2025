#ifndef LDR_SORTER_H
#define LDR_SORTER_H

#include "AbstractColourSorter.h"
#include <Adafruit_NeoPixel.h>

class LdrSorter : public AbstractColorSorter {
private:
    Adafruit_NeoPixel strip;
    byte ldrPin;
    byte ledPin;
    int objectThreshold = 50;

    void setLedColor(byte r, byte g, byte b);
    int readChannel(byte r, byte g, byte b, int samples);

public:
    LdrSorter(byte ldrPin, byte ledPin);
    
    // Implementations of the abstract methods
    void begin() override;
    void getReading(ColorSignature &readStore, int samples) override;
    void calibrate() override;
    char identifyColor() override;
};

#endif