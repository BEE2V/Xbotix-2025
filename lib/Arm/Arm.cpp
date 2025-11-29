#include "Arm.h"

Arm::Arm(uint8_t* stepperPins, int stepsPerRev,
         uint8_t* dcMotorPins, volatile long* externalEncoderCount) {
  
  // Initialize Stepper 
  stepper = new Stepper(stepsPerRev, stepperPins[0], stepperPins[2], stepperPins[1], stepperPins[3]);
  
  // Initialize DC Motor Pins
  pinMotorPWM  = dcMotorPins[0];
  pinMotorDir1 = dcMotorPins[1];
  pinMotorDir2 = dcMotorPins[2];
  
  encoderPos = externalEncoderCount;

  currentGripperSteps = 0;
  targetGripperSteps = 0;
  
  verticalActive = false;
  gripperActive = false;
}

void Arm::begin() {
  pinMode(pinMotorPWM, OUTPUT);
  pinMode(pinMotorDir1, OUTPUT);
  pinMode(pinMotorDir2, OUTPUT);
  
  // Set stepper speed (RPM)
  stepper->setSpeed(15); 
  
  // Assume start at Home
  setHome();
}

void Arm::setMotorSpeed(int maxSpd, int minSpd) {
  motorSpeedMax = maxSpd;
  motorSpeedMin = minSpd;
}

void Arm::setHome() {
  *encoderPos = 0; 
  currentGripperSteps = maxGripperWidth * stepsPerMM_Grip;
  targetGripperSteps = currentGripperSteps;
  targetVerticalPulses = 0;
  verticalActive = false;
  gripperActive = false;
}

// --- Manual Calibration ---
void Arm::manualGripper(int timeMs) {
  unsigned long start = millis();
  unsigned long duration = abs(timeMs);
  
  // If time is Positive, we close (step 1). If negative, we open (step -1).
  // Adjust 1/-1 if your wiring is inverted.
  int dir = (timeMs > 0) ? 1 : -1; 

  // Step continuously until time is up
  while (millis() - start < duration) {
    stepper->step(dir);
  }
}

void Arm::manualVertical(int timeMs) {
  int speed = 100; // Fixed manual speed
  
  if (timeMs > 0) {
    // UP
    digitalWrite(pinMotorDir1, HIGH);
    digitalWrite(pinMotorDir2, LOW);
  } else {
    // DOWN
    digitalWrite(pinMotorDir1, LOW);
    digitalWrite(pinMotorDir2, HIGH);
  }

  analogWrite(pinMotorPWM, speed);
  delay(abs(timeMs));
  
  // STOP
  analogWrite(pinMotorPWM, 0);
  digitalWrite(pinMotorDir1, LOW);
  digitalWrite(pinMotorDir2, LOW);
}

// --- Set Targets (Non-Blocking) ---
void Arm::setGripperWidth(float targetWidth) {
  if (targetWidth < 0) targetWidth = 0;
  if (targetWidth > maxGripperWidth) targetWidth = maxGripperWidth;

  targetGripperSteps = targetWidth * stepsPerMM_Grip;
  gripperActive = true;
}

void Arm::moveToHeight(float targetHeightMM) {
  targetVerticalPulses = targetHeightMM * pulsesPerMM_Vert;
  verticalActive = true;
}

// --- Blocking Implementation ---
void Arm::runTo(float heightMM, float widthMM) {
  moveToHeight(heightMM);
  setGripperWidth(widthMM);
  while (isMoving()) {
    updateArm();
  }
}

void Arm::runHeightTo(float heightMM) {
  moveToHeight(heightMM);
  while (verticalActive) {
    updateArm();
  }
}

void Arm::runGripperTo(float widthMM) {
  setGripperWidth(widthMM);
  while (gripperActive) {
    updateArm();
  }
}

// --- Main Update Loop ---
void Arm::updateArm() {
  if (!verticalActive && !gripperActive) return;

  if (verticalActive) runVerticalLogic();
  if (gripperActive) runGripperLogic();
}

void Arm::runVerticalLogic() {
  long currentPulses = *encoderPos;
  long error = targetVerticalPulses - currentPulses;

  if (abs(error) == 0) {
    digitalWrite(pinMotorDir1, LOW);
    digitalWrite(pinMotorDir2, LOW);
    analogWrite(pinMotorPWM, 0);
    verticalActive = false;
    return;
  }

  int speed = map(abs(error), 0, 50, motorSpeedMin, motorSpeedMax);
  speed = constrain(speed, motorSpeedMin, motorSpeedMax);

  if (error < 0) { 
    digitalWrite(pinMotorDir1, HIGH);
    digitalWrite(pinMotorDir2, LOW);
    analogWrite(pinMotorPWM, speed);
  } else { 
    digitalWrite(pinMotorDir1, LOW);
    digitalWrite(pinMotorDir2, HIGH);
    analogWrite(pinMotorPWM, speed);
  }
}

void Arm::runGripperLogic() {
  if (currentGripperSteps > targetGripperSteps) {
    stepper->step(1); 
    currentGripperSteps--; 
  } 
  else if (currentGripperSteps < targetGripperSteps) {
    stepper->step(-1); 
    currentGripperSteps++;
  } 
  else {
    gripperActive = false;
  }
}

bool Arm::isMoving() {
  return (verticalActive || gripperActive);
}

float Arm::getCurrentHeight() {
  return (*encoderPos) / pulsesPerMM_Vert;
}

float Arm::getCurrentWidth() {
  return currentGripperSteps / stepsPerMM_Grip;
}