#include "LdrSorter.h"

LdrSorter::LdrSorter(byte ldrPin, byte ledPin) 
    : strip(1, ledPin, NEO_GRB + NEO_KHZ800) 
{
    this->ldrPin = ldrPin;
    this->ledPin = ledPin;
    pinMode(ldrPin, INPUT);
    // ledPin is handled by NeoPixel library, but setting output doesn't hurt
    pinMode(ledPin, OUTPUT);
}

void LdrSorter::begin() {
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
}

void LdrSorter::setLedColor(byte r, byte g, byte b) {
    strip.setPixelColor(0, strip.Color(r, g, b));
    strip.show();
    delay(20); // Allow LDR to stabilize
}

int LdrSorter::readChannel(byte r, byte g, byte b, int samples) {
    setLedColor(r, g, b);
    long sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += analogRead(ldrPin);
        delay(2);
    }
    return (int)(sum / samples);
}

void LdrSorter::getReading(ColorSignature &readStore, int samples) {
    readStore.r = readChannel(255, 0, 0, samples);
    readStore.g = readChannel(0, 255, 0, samples);
    readStore.b = readChannel(0, 0, 255, samples);
    setLedColor(0, 0, 0); // Turn off after reading
}

void LdrSorter::calibrate() {
    Serial.println("\n=== LDR/NeoPixel Calibration ===");

    Serial.println("Step 1: EMPTY");
    setLedColor(255,0,0);
    delay(300);
    setLedColor(0,255,0);
    delay(300);
    setLedColor(0,0,255);
    delay(300);
    setLedColor(0,0,0);
    countdown(3);
    getReading(emptySig, 20);
    printSig("Empty", emptySig);

    Serial.println("Step 2: RED Object");
    setLedColor(255,0,0);
    delay(1000);
    setLedColor(0,0,0);
    countdown(3);
    getReading(redSig, 20);
    printSig("Red", redSig);

    Serial.println("Step 3: GREEN Object");
    setLedColor(0,0,255);
    delay(1000);
    setLedColor(0,0,0);
    countdown(3);
    getReading(greenSig, 20);
    printSig("Green", greenSig);

    Serial.println("Calibration Done.");
    delay(1000);
}

char LdrSorter::identifyColor() {
    ColorSignature current;
    getReading(current, 10);

    long currentTotal = current.r + current.g + current.b;
    long emptyTotal = emptySig.r + emptySig.g + emptySig.b;

    if (currentTotal < (emptyTotal + 100)) {
        Serial.print("[LDR] Status: EMPTY | ");
        // printRawLine(current); // Optional debug
        return;
    }

    long distToRed = abs(current.r - redSig.r) + abs(current.g - redSig.g) + abs(current.b - redSig.b);
    long distToGreen = abs(current.r - greenSig.r) + abs(current.g - greenSig.g) + abs(current.b - greenSig.b);

    Serial.print("[LDR] Status: ");
    if (abs(distToRed - distToGreen) < 50) {
        // Serial.print("UNCERTAIN");
        return -1;
    } else if (distToRed < distToGreen) {
        // Serial.print("RED Object");
        return 'r';
    } else {
        // Serial.print("GREEN Object");
        return 'g';
    }
    Serial.println();
}