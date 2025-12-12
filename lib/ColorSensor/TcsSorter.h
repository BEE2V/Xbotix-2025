#ifndef TCS_SORTER_H
#define TCS_SORTER_H

#include "AbstractColourSorter.h"
#include <Adafruit_NeoPixel.h> // Added Library

class TcsSorter : public AbstractColorSorter {
private:
    byte s2Pin;
    byte s3Pin;
    byte outPin;
    
    // --- NEW LED VARIABLES ---
    byte ledPin;
    Adafruit_NeoPixel strip;

    int readRawFrequency(char colorFilter, int samples);

public:
    // Updated Constructor: Now accepts 'ledPin'
    TcsSorter(byte s2, byte s3, byte out, byte ledPin);

    // Helper to change LED color
    void setLedColor(byte r, byte g, byte b);

    void begin() override;
    void getReading(ColorSignature &readStore, int samples) override;
    void calibrate() override;
    char identifyColor() override;
};

#endif