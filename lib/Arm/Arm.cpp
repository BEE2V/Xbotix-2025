#include "Arm.h"

Arm::Arm(uint8_t* stepperPins, int stepsPerRev,
         uint8_t* dcMotorPins, volatile long* externalEncoderCount,
         uint8_t servoPin) {
  
  // Initialize Stepper 
  stepper = new Stepper(stepsPerRev, stepperPins[0], stepperPins[2], stepperPins[1], stepperPins[3]);
  
  // Initialize DC Motor Pins
  pinMotorPWM  = dcMotorPins[0];
  pinMotorDir1 = dcMotorPins[1];
  pinMotorDir2 = dcMotorPins[2];
  
  encoderPos = externalEncoderCount;

  // Initialize Servo
  pinServo = servoPin;
  
  verticalActive = false;
  gripperActive = false;
  wristActive = false;
}

void Arm::begin() {
  pinMode(pinMotorPWM, OUTPUT);
  pinMode(pinMotorDir1, OUTPUT);
  pinMode(pinMotorDir2, OUTPUT);
  
  // Set stepper speed (RPM)
  stepper->setSpeed(15); 

  lastWristUpdate = 0;
  
}

void Arm::setMotorSpeed(int maxSpd, int minSpd) {
  motorSpeedMax = maxSpd;
  motorSpeedMin = minSpd;
}

void Arm::setHome() {
  currentGripperSteps = minGripperWidth * stepsPerMM_Grip;
  targetGripperSteps = currentGripperSteps;

  *encoderPos = 0;
  targetVerticalPulses = 0;

  verticalActive = false;
  gripperActive = false;

  wristServo.attach(pinServo);
  wristServo.write(homeAngle);

  wristActive = false;
  
  currentWristAngle = homeAngle;
  targetWristAngle = homeAngle;

  runWristTo(topAngle);
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
  int speed = 200; // Fixed manual speed
  
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

void Arm::setHeight(float targetHeightMM) {
  targetVerticalPulses = targetHeightMM * pulsesPerMM_Vert;
  verticalActive = true;
}

void Arm::setWristAngle(float angleDeg) {
  if (angleDeg < wristAngleMin) angleDeg = wristAngleMin;
  if (angleDeg > wristAngleMax) angleDeg = wristAngleMax;

  targetWristAngle = angleDeg;
  wristActive = true;
}

// --- Blocking Implementation ---
void Arm::runTo(float heightMM, float widthMM) {
  setHeight(heightMM);
  setGripperWidth(widthMM);
  while (isMoving()) {
    updateArm();
  }
}

void Arm::runHeightTo(float heightMM) {
  setHeight(heightMM);
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

void Arm::runWristTo(int angleDeg) {

  setWristAngle(angleDeg);

  wristServo.attach(pinServo);
  while (wristActive) {
    updateArm();
  }
    
}

// --- Main Update Loop ---
void Arm::updateArm() {
  if (!verticalActive && !gripperActive && !wristActive) return;

  if (verticalActive) runVerticalLogic();
  if (gripperActive) runGripperLogic();
  if (wristActive) runWristLogic();
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

  int speed = map(abs(error), 0, 500, motorSpeedMin, motorSpeedMax);
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
    stepper->step(-1); 
    currentGripperSteps--; 
  } 
  else if (currentGripperSteps < targetGripperSteps) {
    stepper->step(1); 
    currentGripperSteps++;
  } 
  else {
    gripperActive = false;
  }
}

void Arm::runWristLogic() {
  if(millis() - lastWristUpdate < angleDelay){
    return;
  }
    
  if(currentWristAngle < targetWristAngle) {
    currentWristAngle++;
    wristServo.write(currentWristAngle);
  }else if(currentWristAngle > targetWristAngle) {
    currentWristAngle--;
    wristServo.write(currentWristAngle);
  }else{
    wristActive = false;
  }
  lastWristUpdate = millis(); 
  
}

bool Arm::isMoving() {
  return (verticalActive || gripperActive || wristActive);
}

float Arm::getCurrentHeight() {
  return (*encoderPos) / pulsesPerMM_Vert;
}

float Arm::getCurrentWidth() {
  return currentGripperSteps / stepsPerMM_Grip;
}