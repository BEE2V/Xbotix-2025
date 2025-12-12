#include "Driver.h"

// Constructor implementation
// Expects pin arrays in order: {PWM, IN1, IN2}
Driver::Driver(volatile long* enc1_ptr, uint8_t* pins1, 
               volatile long* enc2_ptr, uint8_t* pins2) {
  
  // Initialize Motor 1 Context
  m1.encPosPtr = enc1_ptr;
  m1.pinPWM = pins1[0];
  m1.pinIN1 = pins1[1];
  m1.pinIN2 = pins1[2];
  m1.countsPerRev = countsPerRev_m1; // From your original M1 code
  
  // Initialize Motor 2 Context
  m2.encPosPtr = enc2_ptr;
  m2.pinPWM = pins2[0];
  m2.pinIN1 = pins2[1];
  m2.pinIN2 = pins2[2];
  m2.countsPerRev = countsPerRev_m2; // From your original M2 code

  // Initialize Pins
  pinMode(m1.pinPWM, OUTPUT); pinMode(m1.pinIN1, OUTPUT); pinMode(m1.pinIN2, OUTPUT);
  pinMode(m2.pinPWM, OUTPUT); pinMode(m2.pinIN1, OUTPUT); pinMode(m2.pinIN2, OUTPUT);

  // Initialize State Vars
  m1.prevT = micros(); m1.posPrev = 0; m1.vFilt = 0; m1.vPrev = 0; m1.eIntegral = 0; m1.targetRPM = 0;
  m2.prevT = micros(); m2.posPrev = 0; m2.vFilt = 0; m2.vPrev = 0; m2.eIntegral = 0; m2.targetRPM = 0;
}

void Driver::resetVars(){
  // Initialize State Vars
  m1.prevT = micros(); m1.posPrev = 0; m1.vFilt = 0; m1.vPrev = 0; m1.eIntegral = 0; m1.targetRPM = 0;
  m2.prevT = micros(); m2.posPrev = 0; m2.vFilt = 0; m2.vPrev = 0; m2.eIntegral = 0; m2.targetRPM = 0;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    *m1.encPosPtr = 0;
    *m2.encPosPtr = 0;
  }
}

void Driver::computePID(MotorContext &m) {
  // 1. Read Encoder Atomically
  long pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = *m.encPosPtr; // Dereference the pointer
  }

  // 2. Time Delta
  long currT = micros();
  float deltaT = ((float) (currT - m.prevT)) / 1.0e6;
  
  // Safety check to prevent division by zero
  if (deltaT <= 0) return; 

  // 3. Velocity Calculation
  float velocity = (pos - m.posPrev) / deltaT;
  m.posPrev = pos;
  m.prevT = currT;

  // 4. Convert to RPM
  float vRPM = velocity / m.countsPerRev * 60.0;

  // 5. Low Pass Filter
  // vFilt = 0.854*vFilt + 0.0728*vRaw + 0.0728*vPrevRaw
  m.vFilt = 0.854 * m.vFilt + 0.0728 * vRPM + 0.0728 * m.vPrev;
  m.vPrev = vRPM;

  // 6. PID Calculation
  // Note: Your original code inverted Target for M1 (-1 * target). 
  // I have standardized it here: Error = Target - Measured.
  // If your motor spins backwards, flip the pin definition or multiply target by -1 here.
  
  float e = m.targetRPM - m.vFilt;
  
  m.eIntegral = m.eIntegral + e * deltaT;
  float u = kp * e + ki * m.eIntegral;

  // 7. Set Motor Hardware
  int dir = 1;
  if (u < 0) dir = -1;
  
  int pwr = (int) fabs(u);
  if(pwr > 255) pwr = 255;

  setMotorHardware(dir, pwr, m.pinPWM, m.pinIN1, m.pinIN2);
}

void Driver::setMotorHardware(int dir, int pwmVal, int pwmPin, int in1Pin, int in2Pin) {
  analogWrite(pwmPin, pwmVal);
  if(dir == 1){ 
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  }
  else if(dir == -1){
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }
  else{
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);    
  }
}

void Driver::runMotors(float targetRPM1, float targetRPM2) {
  m1.targetRPM = targetRPM1;
  m2.targetRPM = targetRPM2;
}

void Driver::updateSpeed() {
  computePID(m1);
  computePID(m2);
}


void Driver::applyLinePid(float correction, float base = -1){
  
  float leftSpeed;
  float rightSpeed;
  
  if(base > 0){
    leftSpeed = base + correction;
    rightSpeed = base - correction;
  }else{
    leftSpeed = leftBase + correction;
    rightSpeed = rightBase - correction;
  } 
  
  if (leftSpeed < 0) {
      leftSpeed = 0;
  }

  if (rightSpeed < 0) {
      rightSpeed = 0;
  }

  if (leftSpeed >= leftMax) {
      leftSpeed = leftMax;
  }

  if (rightSpeed >= rightMax) {
      rightSpeed = rightMax;
  }

  runMotors(rightSpeed, leftSpeed);

}

void Driver::applyEncPid(float correction, float base){

  if(correction > correctionMax) correction = correctionMax;
  if(correction < correctionMax * -1) correction = correctionMax * -1;

  float leftSpeed = abs(base) - correction;
  float rightSpeed = abs(base) + correction;

  if(base >= 0) runMotors(rightSpeed, leftSpeed);
  else runMotors(-leftSpeed, -rightSpeed);

}

void Driver::stop(int del){
  setMotorHardware(1,0, m1.pinPWM, m1.pinIN1, m1.pinIN2);
  setMotorHardware(1,0,m2.pinPWM,m2.pinIN1,m2.pinIN2);
  delay(del);
  resetVars();
} 

void Driver::brake(){
  setMotorHardware(1,0, m1.pinPWM, m1.pinIN1, m1.pinIN2);
  setMotorHardware(1,0,m2.pinPWM,m2.pinIN1,m2.pinIN2);
}

void Driver::turnRight(float spd){
  runMotors(-1.0 * spd, spd);
  updateSpeed();
}

void Driver::turnLeft(float spd){
  runMotors(spd, -1.0 * spd);
  updateSpeed();

}

float Driver::getLeftDistance(){
  long pos;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = *m2.encPosPtr; // Dereference the pointer
  }
  return (pos / countsPerRev_m2) * 2 * 3.14 * 23;
}

float Driver::getRightDistance(){
  long pos;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = *m1.encPosPtr; // Dereference the pointer
  }

  return (pos / countsPerRev_m1) * 2 * 3.14 * 23;
}