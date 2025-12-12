#include <Tof.h>
#include <Wire.h>

Tof::Tof(int *settings){
    //xshut = settings[0];
    longRange = settings[0];
    highAccuracy = settings[1];

}

void Tof::init(){
    //digitalWrite(xshut, HIGH);
    Wire.begin();

    sensor.setTimeout(500);
    if (!sensor.init())
    {
        Serial.println("Failed to detect and initialize sensor!");
        while (1) {}
    }

    // Start continuous back-to-back mode (take readings as
    // fast as possible).  To use continuous timed mode
    // instead, provide a desired inter-measurement period in
    // ms (e.g. sensor.startContinuous(100)).
    sensor.startContinuous();

}

void Tof::shut(){
    digitalWrite(xshut, LOW);
}

void Tof::read(){
    int data = sensor.readRangeContinuousMillimeters();

    if(data < 220){
        //Serial.print(data);
        reading = data;
    }
    else{
        //Serial.print("Out of range");
        reading = -1;
        
    }


}