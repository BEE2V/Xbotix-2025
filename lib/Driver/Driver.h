#ifndef DRIVER_H
#define DRIVER_H

#include <Arduino.h>
#include <util/atomic.h>

// Internal struct to hold state for a single motor
struct MotorContext {
  // Pins
  uint8_t pinPWM;
  uint8_t pinIN1;
  uint8_t pinIN2;

  // Pointer to the external encoder count
  volatile long* encPosPtr;

  // PID Internal State
  long prevT;
  long posPrev;
  float vFilt;
  float vPrev;
  float eIntegral;
  float targetRPM;
  
  // Motor Specific Constants
  float countsPerRev;
};

class Driver {

private:

  float countsPerRev_m1 = 225.0;
  float countsPerRev_m2 = 225.0;

  const float leftBase = 80.0;
  const float rightBase = 80.0;

  float leftMax = 180.0;
  float rightMax = 180.0;

  const float correctionMax = 100.0;

  MotorContext m1;
  MotorContext m2;

  void computePID(MotorContext &m);
  
  void setMotorHardware(int dir, int pwmVal, int pwmPin, int in1Pin, int in2Pin);

public:

  float kp = 0.8;    // 0.7
  float ki = 7.5;    //15.0
  
  Driver(volatile long* enc1_ptr, uint8_t* pins1, 
         volatile long* enc2_ptr, uint8_t* pins2);

  void resetVars();

  void runMotors(float targetRPM1, float targetRPM2);

  void updateSpeed();


  void applyLinePid(float correction, float base = -1);

  void applyEncPid(float correction, float base);


  void stop(int del = 1000);

  void brake();

  void turnRight(float speed);

  void turnLeft(float speed);

  float getLeftDistance();

  float getRightDistance();

};

#endif