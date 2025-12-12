#include "TcsSorter.h"

// 1. Updated Constructor
TcsSorter::TcsSorter(byte s2, byte s3, byte out, byte ledPin) 
    : strip(1, ledPin, NEO_GRB + NEO_KHZ800) // Initialize NeoPixel
{
    this->s2Pin = s2;
    this->s3Pin = s3;
    this->outPin = out;
    this->ledPin = ledPin;

    pinMode(s2Pin, OUTPUT);
    pinMode(s3Pin, OUTPUT);
    pinMode(outPin, INPUT);
    // Note: No need to pinMode(ledPin) for NeoPixel, the library handles it
}

void TcsSorter::begin() {
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
}

// 2. New Helper Function
void TcsSorter::setLedColor(byte r, byte g, byte b) {
    strip.setPixelColor(0, strip.Color(r, g, b));
    strip.show();
    delay(20); // Small delay for light to stabilize
}

int TcsSorter::readRawFrequency(char colorFilter, int samples) {
    // Set Sensor Filter
    switch(colorFilter) {
      case 'R': digitalWrite(s2Pin, LOW);  digitalWrite(s3Pin, LOW);  break;
      case 'G': digitalWrite(s2Pin, HIGH); digitalWrite(s3Pin, HIGH); break;
      case 'B': digitalWrite(s2Pin, LOW);  digitalWrite(s3Pin, HIGH); break;
    }
    
    // Note: We don't delay here anymore because setLedColor has a delay
    
    long sum = 0;
    for (int i = 0; i < samples; i++) {
      long duration = pulseIn(outPin, LOW, 20000); 
      if (duration == 0) duration = 20000; 
      sum += (10000 / duration); 
      delay(2);
    }
    return (int)(sum / samples);
}

// 3. Updated Reading Logic (Syncs LED with Sensor)
void TcsSorter::getReading(ColorSignature &readStore, int samples) {
    // Read Red (Turn LED Red, Set Filter Red)
    //setLedColor(255, 0, 0);
    readStore.r = readRawFrequency('R', samples);

    // Read Green (Turn LED Green, Set Filter Green)
    //setLedColor(0, 255, 0);
    readStore.g = readRawFrequency('G', samples);

    // Read Blue (Turn LED Blue, Set Filter Blue)
    //setLedColor(0, 0, 255);
    readStore.b = readRawFrequency('B', samples);
    
    // Turn LED Off
    setLedColor(0, 0, 0);
}

void TcsSorter::calibrate() {
    Serial.println("\n=== TCS230 Calibration ===");

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

char TcsSorter::identifyColor() {
    ColorSignature current;
    getReading(current, 10);

    long currentTotal = current.r + current.g + current.b;
    long emptyTotal = emptySig.r + emptySig.g + emptySig.b;

    // 1. Check if Empty
    if (currentTotal < (emptyTotal + 10)) {
        return -1; // No Object
    }

    long distToRed = abs(current.r - redSig.r) + abs(current.g - redSig.g) + abs(current.b - redSig.b);
    long distToGreen = abs(current.r - greenSig.r) + abs(current.g - greenSig.g) + abs(current.b - greenSig.b);

    // 2. Confusion Check
    if (abs(distToRed - distToGreen) < 20) {
        return -1; // Uncertain
    } 
    
    // 3. Color Decision
    if (distToRed < distToGreen) {
        return 'r'; 
    } else {
        return 'g'; 
    }
}