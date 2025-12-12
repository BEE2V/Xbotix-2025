#include <Sonic.h>

long mm,duration;

int Sonic::readDistance(){
    
    digitalWrite(t, LOW);
    delayMicroseconds(5);
    digitalWrite(t, HIGH);
    delayMicroseconds(10);
    digitalWrite(t, LOW);

    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(e, INPUT);
    duration = pulseIn(e, HIGH, 2000);

    // Convert the time into a distance
    mm = (duration/2) * 0.343;     // Divide by 29.1 or multiply by 0.0343
    
    // if(mm < 0){
    //     return mm;
    // }else{
    //     if(t == 32){
    //         return readLeftLox();
    //     }else{
    //         return readRightLox();
    //     }
    // }

    delay(5);

    if(mm >= max * 10){
        return 0;
    }
    return mm;
}

bool Sonic::wallFound(){
    int i = 10;
    int found = 0;

    while(i-- >= 0){
        int distance = readDistance();
        if(distance < 100 && distance != 0){
            found++;
        }else{
            found--;
        }
    }
    return found > 0 ? true:false;
}