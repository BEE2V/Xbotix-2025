#include <PID.h>
#include <Arduino.h>

float PID::getEncoderCorrection(int error){
    double p = error * eP;
    double d = (error - prevEncError) *eD;

    prevEncError = error;

    int correction = (int)(p + d);

    return correction;
}

float PID::getLineCorrection(int error){
    totalLineError += error;

    double p;
    double i;
    double d;

    p = error * P;
    i = totalLineError * I;
    d = (error - prevLineError) * D;
    
    prevLineError = error;

    float correction = (p + i + d);

    return correction;
}


float PID::getLineCorrectionVC(int error){
    totalLinevError += error;

    double p;
    double i;
    double d;

    p = error * vP;
    i = totalLinevError * vI;
    d = (error - prevLinevError) * vD;
    
    prevLinevError = error;

    float correction = (p + i + d);

    return correction;
}

float PID::getEncCorrectionVC(int error){
    totalEncvError += error;

    double p;
    double i;
    double d;

    p = error * veP;
    i = totalEncvError * veI;
    d = (error - prevEncvError) * veD;
    
    prevEncvError = error;

    float correction = (p + i + d);

    return correction;
}


int PID::getSonicCorrection(int error){
    double p = error * sP;
    double d = (error - prevSonicError) *sD;

    prevSonicError = error;

    int correction = (int)(p + d);

    return correction;
}

int PID::getWallCorrection(int error){
    double p = error * wP;
    double d = (error - prevWallError) *wD;

    prevWallError = error;

    int correction = (int)(p + d);

    return correction;
}