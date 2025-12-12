#ifndef ABSTRACT_COLOR_SORTER_H
#define ABSTRACT_COLOR_SORTER_H

#include <Arduino.h>
#include <EEPROM.h>

struct ColorSignature {
    int r;
    int g;
    int b;
};

// A unique ID to check if EEPROM has valid data for this sensor
// We use a small header struct
struct CalibrationHeader {
    byte magicNumber; // We will use 0xAA as our "valid" flag
    byte sensorType;  // Optional: 1 for LDR, 2 for TCS
};

class AbstractColorSorter {
protected:
    ColorSignature emptySig;
    ColorSignature redSig;
    ColorSignature greenSig;

    // EEPROM Address helper
    int eepromStartAddress;

    void printSig(String name, ColorSignature s) {
        Serial.print("  [Saved] "); Serial.print(name);
        Serial.print(" -> R:"); Serial.print(s.r);
        Serial.print(" G:"); Serial.print(s.g);
        Serial.print(" B:"); Serial.println(s.b);
    }

    void countdown(int seconds) {
        for (int i = seconds; i > 0; i--) {
            Serial.print(i); Serial.print("... ");
            delay(1000);
        }
        Serial.println("GO!");
    }

public:
    virtual ~AbstractColorSorter() {} 

    // Pure virtual methods
    virtual void begin() = 0;
    virtual void getReading(ColorSignature &readStore, int samples) = 0;
    virtual void calibrate() = 0;
    virtual char identifyColor() = 0;

    // --- NEW EEPROM METHODS ---
    
    // Set where in EEPROM this sensor lives
    void setEEPROMAddress(int address) {
        this->eepromStartAddress = address;
    }

    void saveCalibration() {
        int addr = eepromStartAddress;
        
        // 1. Write Header (Magic Number 0xAA)
        EEPROM.put(addr, 0xAA); 
        addr += sizeof(byte);

        // 2. Write Data
        EEPROM.put(addr, emptySig); 
        addr += sizeof(ColorSignature);
        
        EEPROM.put(addr, redSig);   
        addr += sizeof(ColorSignature);
        
        EEPROM.put(addr, greenSig); 
        
        Serial.println(">> Calibration Saved to EEPROM!");
    }

    // Returns TRUE if valid data was found and loaded
    bool loadCalibration() {
        int addr = eepromStartAddress;
        byte magic;
        
        // 1. Check Header
        EEPROM.get(addr, magic);
        if (magic != 0xAA) {
            Serial.println(">> No valid EEPROM data found.");
            return false; // Invalid data
        }
        addr += sizeof(byte);

        // 2. Read Data
        EEPROM.get(addr, emptySig);
        addr += sizeof(ColorSignature);

        EEPROM.get(addr, redSig);
        addr += sizeof(ColorSignature);

        EEPROM.get(addr, greenSig);

        Serial.println(">> Calibration Loaded from EEPROM:");
        printSig("Empty", emptySig);
        printSig("Red", redSig);
        printSig("Green", greenSig);
        return true;
    }
};

#endif