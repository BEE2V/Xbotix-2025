#include <Arduino.h>
#include "Arm.h"
#include "Tof.h"
#include "Driver.h"
#include "SensorPanel.h"
#include "PID.h"
#include <MotorDriver.h>
#include <Sonic.h>
#include <LdrSorter.h>
#include <TcsSorter.h>
#include <Feeder.h>

const int EEPROM_ADDR_TOWER = 100; // Start address in EEPROM
const int EEPROM_ADDR_BALL = 200; // Start address in EEPROM

// --- CONFIGURATION ---
const int STEPS_PER_REV = 2048;

int bOutTower[3];
int bOutBall[3];

// --- Encoder Pins ---
#define ENC_A       18 // Interrupt Pin
#define ENC_B       46 // Digital Pin

// --- Motor Encoder Pins ---
#define M1_ENCODER_A 2
#define M1_ENCODER_B 15
#define M2_ENCODER_A 3
#define M2_ENCODER_B 14

#define EEPROM_ADDR_LDR  0   
#define EEPROM_ADDR_TCS  50  

#define CALIBRATE_BTN_PIN A1
bool forceCalibrate = false;

volatile long encoderCount = 0;

volatile long pos_i_M1 = 0;
volatile long pos_i_M2 = 0;

LdrSorter ldrSensor(A0, 11);
TcsSorter tcsSensor(9, 10, A2, 11);

Sonic leftSonic(44, 45, 20);
Sonic rightSonic(42, 43, 20);

Tof frontLox{
  const_cast<int *>((const int[]){0, 0})
};

Feeder feeder(
  const_cast<uint8_t *>((const uint8_t[]) {A10, A11, A12, A13}), 
  STEPS_PER_REV, 
  8,
  7
);

Arm robot(
  const_cast<uint8_t *>((const uint8_t[]) {53, 51, 52, 50}), 
  STEPS_PER_REV, 
  const_cast<uint8_t *>((const uint8_t[]) {13, 16, 17}), 
  &encoderCount,
  6
);

Driver driver(
  &pos_i_M1, 
  const_cast<uint8_t *>((const uint8_t[]) {4, 38, 39}), // M1 Pins
  &pos_i_M2, 
  const_cast<uint8_t *>((const uint8_t[]) {5, 40, 41})  // M2 Pins
);

MotorDriver mdriver(
  const_cast<int *>((const int[]) {5, 40, 41}),  // M2 Pins
  const_cast<int *>((const int[]) {4, 38, 39}) // M1 Pins
);

SensorPanel qtr(const_cast<uint8_t *>((const uint8_t[]) {27, 28, 26, 23, 25, 31, 30, 24}));

PID pid;

void saveTowerCode(char* colors) {
  for (int i = 0; i < 3; i++) {
    byte bitValue = 0;

    // Logic: Green = 1, Anything else (Red) = 0
    if (colors[i] == 'G' || colors[i] == 'g') {
      bitValue = 1;
    } else {
      bitValue = 0;
    }

    // Save the 1 or 0 to EEPROM
    EEPROM.update(EEPROM_ADDR_TOWER + i, bitValue);
  }
}

void getTowerCode(int* binaryOutputTower) {
  for (int i = 0; i < 3; i++) {
    // Read the byte (which will be 1 or 0)
    binaryOutputTower[i] = EEPROM.read(EEPROM_ADDR_TOWER + i);
  }
}

void saveBallCode(char* balls) {
  byte greenCount = 0;

  // Loop through all 5 balls to count Greens
  for (int i = 0; i < 5; i++) {
    if (balls[i] == 'G' || balls[i] == 'g') {
      greenCount++;
    }
  }

  // Save the total count (e.g., 3) to a single EEPROM slot
  // We only need one slot because 0-5 fits easily in one byte.
  EEPROM.update(EEPROM_ADDR_BALL, greenCount);
  
  Serial.print("Saved count: ");
  Serial.println(greenCount);
}

void getBallCode(int* binaryOutputBall) {
  // 1. Read the count from memory (e.g., 3)
  byte count = EEPROM.read(EEPROM_ADDR_BALL);

  // 2. Convert Decimal to Binary [MSB, Middle, LSB]
  
  // Bit 2 (4s place): Check if bit 2 is high
  binaryOutputBall[0] = (count >> 2) & 1; 
  
  // Bit 1 (2s place): Check if bit 1 is high
  binaryOutputBall[1] = (count >> 1) & 1; 
  
  // Bit 0 (1s place): Check if bit 0 is high
  binaryOutputBall[2] = (count >> 0) & 1; 
}

// --- Interrupt Service Routine ---

void readEncoderLift() {
  if (digitalRead(ENC_B) == LOW) {
    encoderCount--;
  } else {
    encoderCount++;
  }
}

void readEncoderM1(){
  int b = digitalRead(M1_ENCODER_B);
  if(b > 0){
    pos_i_M1--;
  } else {
    pos_i_M1++;
  }
}

void readEncoderM2(){
  int b = digitalRead(M2_ENCODER_B);
  if(b > 0){
    pos_i_M2++;
  } else {
    pos_i_M2--;
  }
}

void manualCalibrationArm(){
  // --- MANUAL CALIBRATION LOOP ---
  Serial.println("=== CALIBRATION MODE ===");
  Serial.println("Type 'g' for Gripper Mode");
  Serial.println("Type 'l' for Lift Mode");
  Serial.println("Type 'x' to Set Home & Start");
  Serial.println("Enter time (ms) to move: e.g. 100 or -100");
  
  char mode = ' '; // Current mode ('g' or 'l')
  
  while (true) {
    if (Serial.available() > 0) {
      // Check if it's a command character (g, l, x)
      char c = Serial.peek();
      
      if (isAlpha(c)) {
        char cmd = Serial.read();
        if (cmd == 'x') {
          Serial.println("Exiting Calibration...");
          break; // Exit the loop
        }
        else if (cmd == 'g' || cmd == 'l') {
          mode = cmd;
          Serial.print("Mode Set: ");
          Serial.println(mode == 'g' ? "GRIPPER" : "LIFT");
        }
        // Flush buffer
        while(Serial.available()) Serial.read();
      }
      else {
        // It's a number (time duration)
        int timeMs = Serial.parseInt();
        if (timeMs != 0) {
          Serial.print("Moving for "); Serial.print(timeMs); Serial.println(" ms");
          
          if (mode == 'g') {
            robot.manualGripper(timeMs);
          } 
          else if (mode == 'l') {
            robot.manualVertical(timeMs);
          } 
          else {
            Serial.println("Error: Select mode 'g' or 'l' first.");
          }
        }
        // Flush newline/garbage
        while(Serial.available() && Serial.peek() < 33) Serial.read();
      }
    }
  }
}

void switchCalibrationArm(){
  // --- BUTTON CONFIGURATION ---
  const int MODE_BTN = A1;  // Change to your pin
  const int INC_BTN  = 35;  // Change to your pin
  const int DEC_BTN  = 32;  // Change to your pin

  pinMode(MODE_BTN, INPUT_PULLUP);
  pinMode(INC_BTN, INPUT_PULLUP);
  pinMode(DEC_BTN, INPUT_PULLUP);

  // --- MANUAL CALIBRATION LOOP ---
  Serial.println("=== CALIBRATION MODE (BUTTONS) ===");
  Serial.println("Single Press Mode Btn: Toggle Lift/Gripper");
  Serial.println("Double Press Mode Btn: Save & Exit");
  Serial.println("Hold INC/DEC to move");

  char mode = 'l'; // Default mode ('l' or 'g')
  Serial.println("Current Mode: LIFT");

  while (true) {
    
    // --- 1. HANDLE MODE BUTTON (Single vs Double Press) ---
    if (digitalRead(MODE_BTN) == LOW) {
      delay(50); // Debounce
      if (digitalRead(MODE_BTN) == LOW) {
        
        // Wait for release
        while(digitalRead(MODE_BTN) == LOW); 
        
        // Check for Double Press: Wait 300ms to see if pressed again
        bool doublePress = false;
        unsigned long timer = millis();
        while (millis() - timer < 300) {
          if (digitalRead(MODE_BTN) == LOW) {
            doublePress = true;
            break;
          }
        }

        if (doublePress) {
          // --- DOUBLE PRESS DETECTED: EXIT ---
          Serial.println("Exiting Calibration...");
          while(digitalRead(MODE_BTN) == LOW); // Wait for release
          break; // Break the main while loop
        } 
        else {
          // --- SINGLE PRESS DETECTED: TOGGLE MODE ---
          if (mode == 'l') {
            mode = 'g';
            Serial.println("Mode Switched: GRIPPER");
          } else {
            mode = 'l';
            Serial.println("Mode Switched: LIFT");
          }
        }
      }
    }

    // --- 2. HANDLE MOVEMENT (Push and Hold) ---
    // We send small time increments (e.g. 50ms) repeatedly while held
    
    int moveTime = 0;

    if (digitalRead(INC_BTN) == LOW) {
      moveTime = 50; // Move Positive
    } 
    else if (digitalRead(DEC_BTN) == LOW) {
      moveTime = -50; // Move Negative
    }

    if (moveTime != 0) {
      // Execute move based on current mode
      if (mode == 'g') {
        robot.manualGripper(moveTime);
      } 
      else if (mode == 'l') {
        robot.manualVertical(moveTime);
      }
      // Small delay to prevent overwhelming the motor controller 
      // and to allow the "hold" loop to feel smooth
      delay(10); 
    }
  }
}

void calibrateColourSensors(){
  const int MODE_BTN = A1;
  
  Serial.println("\n[Sensor 1: LDR]");
    if (forceCalibrate || !ldrSensor.loadCalibration()) {
        Serial.println("Calibration required...");
        ldrSensor.calibrate();
        ldrSensor.saveCalibration();
    }

    while(true){
      if (digitalRead(MODE_BTN) == LOW) {
        delay(50); // Debounce
        if (digitalRead(MODE_BTN) == LOW) {
          
          // Wait for release
          while(digitalRead(MODE_BTN) == LOW); 
          break;
        }
      }
    }

    // --- SENSOR 2 SETUP ---
    Serial.println("\n[Sensor 2: TCS]");
    if (forceCalibrate || !tcsSensor.loadCalibration()) {
        Serial.println("Calibration required...");
        tcsSensor.calibrate();
        tcsSensor.saveCalibration();
    }

    Serial.println("\n>>> ALL SYSTEMS READY <<<");
}

void keepAlive(){
  robot.updateArm();
}

void turnRightTillMiddle(){

  float t = millis();
  while(millis() - t < 500){
    keepAlive();
    driver.turnRight(60);
  } 
  
  qtr.read();
  while(qtr.panelReading[3] == 1){
    keepAlive();
    driver.turnRight(60);
    qtr.read();
  }
  
  qtr.read();
  while(qtr.panelReading[3] != 1){
    keepAlive();
    driver.turnRight(60);
    qtr.read();
  }
  
}

void turnLeftTillMiddle(){
  
  float t = millis();
  while(millis() - t < 500){
    keepAlive();
    driver.turnLeft(60);
  } 

  qtr.read();
  while(qtr.panelReading[4] == 1){
    keepAlive();
    driver.turnLeft(60);
    qtr.read();
  }
  
  qtr.read();
  while(qtr.panelReading[4] != 1){
    keepAlive();
    driver.turnLeft(60);
    qtr.read();
  }
  
}

void turn90(float speed, char dir = 'l'){
  driver.resetVars();
  float targetDist = 210;

  while(abs(driver.getLeftDistance()) + abs(driver.getRightDistance()) / 2.0 < targetDist){
    keepAlive();
    dir == 'l'? driver.runMotors(speed,-speed) : driver.runMotors(-speed, speed);
    driver.updateSpeed();
  }
}

void turn180(float speed, char dir = 'l'){
  driver.resetVars();
  float targetDist = 415;

  while(abs(driver.getLeftDistance()) + abs(driver.getRightDistance()) / 2.0 < targetDist){
    keepAlive();
    dir == 'l'? driver.runMotors(speed,-speed) : driver.runMotors(-speed, speed);
    driver.updateSpeed();
  }
}

void goStraightFor(float distanceMM, float speed){
  driver.resetVars();
  float targetDist = distanceMM;

  while((driver.getLeftDistance() + driver.getRightDistance()) / 2.0 < targetDist){
    keepAlive();
    float err = driver.getLeftDistance() - driver.getRightDistance();
    float correction = pid.getEncCorrectionVC(err);
    driver.applyEncPid(correction, speed);
    driver.updateSpeed();
  }
}

void goStraightBackwardFor(float distanceMM, float speed){
  driver.resetVars();
  float targetDist = distanceMM;
  
  float reverseSpeed = -1.0 * abs(speed); 

  while(abs((driver.getLeftDistance() + driver.getRightDistance()) / 2.0) < targetDist){
    keepAlive();
    
    float err = driver.getLeftDistance() - driver.getRightDistance();
    float correction = pid.getEncCorrectionVC(err);
    
    driver.applyEncPid(correction, reverseSpeed);
    driver.updateSpeed();
  }
}

void goStraight(float speed){

  float err = driver.getLeftDistance() - driver.getRightDistance();
  float correction = pid.getEncCorrectionVC(err);
  driver.applyEncPid(correction, speed);
  driver.updateSpeed();
  
}

void goStraightBackward(float reverseSpeed){
  float err = driver.getLeftDistance() - driver.getRightDistance();
  float correction = pid.getEncCorrectionVC(err);
  
  driver.applyEncPid(correction, -reverseSpeed);
  driver.updateSpeed();
}


void goStraightEnc(int speed){

  int errEncoder = pos_i_M1 - pos_i_M2;

  int correction = pid.getEncoderCorrection(errEncoder);
  mdriver.applyEncoderPid(correction, speed);

  //driver.forward(sonicLeftBase,sonicRightBase);
}

float tempLeftDist = 0;
float tempRightDist = 0;

long lastupdateDistTime = 0;

float lastLeftDist = 0;
float lastRightDist = 0;

bool enteredCircle = false;

bool approachingRamp = false;
int rampCount = 0;

void lineFollow(){

  keepAlive();
  float pushDis = 130;

  qtr.read();
  if (qtr.pattern == 1) {
    
    if(approachingRamp){
      int correction = pid.getLineCorrection(qtr.error);
      mdriver.applyLinePid(correction, 120);
    }else{
      int correction = pid.getLineCorrection(qtr.error);
      mdriver.applyLinePid(correction);
    }

    if(millis() - lastupdateDistTime > 150){
      lastupdateDistTime = millis();
      tempLeftDist = driver.getLeftDistance() - lastLeftDist; 
      tempRightDist = driver.getRightDistance() - lastRightDist;

      lastLeftDist = driver.getLeftDistance();
      lastRightDist = driver.getRightDistance();
    }

  }else{
    if(approachingRamp){
      goStraightFor(100, 60);
      approachingRamp = false;
      return;
    }
    char pattern = qtr.pattern;

    bool left = pattern == 'L';
    bool right = pattern == 'R';
    bool t = pattern == 'T';

    driver.resetVars();
    float leftDist = driver.getLeftDistance() + pushDis;
    float rightDist = driver.getRightDistance() + pushDis;

    while(driver.getRightDistance() <= rightDist || driver.getRightDistance() <= leftDist){

      keepAlive();
      goStraight(80.0);
      qtr.read();

      if (qtr.pattern == 'L') {
        left = true; 
      } else if (qtr.pattern == 'R') {
        right = true;
      } else if (qtr.pattern == 'T') {
        t = true;
      }

    }

    if (t || (left && right)) {
      pattern = 'T';
    } else if (left) {
      pattern = 'L';
    } else if (right) {
      pattern = 'R';
    } else {
      pattern = 0;
    }
    
    qtr.read();
    char newPattern = qtr.pattern;

    if(newPattern == 'T'){
      driver.stop(99999);
    }else if(newPattern == 1){
      if(pattern == 'T'){
        enteredCircle = true;
        return;
      }
    }

    driver.stop(500);    

    if (pattern == 'L') {
      turnLeftTillMiddle();
    } else if (pattern == 'R') {
      turnRightTillMiddle();
    }else if(pattern == 'T'){
      turnLeftTillMiddle();
    }else{
      turnLeftTillMiddle();
    }

  }

}

int state;
int preState;

int sideGapRight = 100;
int sideGapLeft = 100;

int jump = 0;

int wallSpeed = 60;

int leftCount = 0;
int rightCount = 0;

bool foundLeft = false;
bool foundRight = false;

int r = 0;
int l = 0;

void wallFollow(){

  if(leftSonic.readDistance() != 0 && rightSonic.readDistance() != 0){
    state = 1;
    if (state != preState)
    {
      jump = 10;
      pid.prevWallError = 0;
      pos_i_M1 = 0;
      pos_i_M2 = 0;
      
    }
    if (jump>0)
    {
      goStraightEnc(wallSpeed);
      jump = jump-1;
      
    }
    else
    {
      r = rightSonic.readDistance();
      l = leftSonic.readDistance();
      int err = l - r;
      int correction = pid.getWallCorrection(err);
      mdriver.applyWallPid(correction * -1);

    }

  }
  else if(leftSonic.readDistance() == 0 && rightSonic.readDistance() != 0){
      state = 2;

      if (state != preState)
      {
        jump = 10;
        pid.prevSonicError = 0;
        pos_i_M1 = 0;
        pos_i_M2 = 0;
      }
      if (jump>0)
      {
        goStraightEnc(wallSpeed);
        jump = jump-1;
        leftCount++;

        if(leftCount >= 5){
          foundLeft = true;
        }
      }
      else
      {
        int err = sideGapRight - rightSonic.readDistance();
        int correction = pid.getSonicCorrection(err);
        mdriver.applySonicPid(correction * -1);
      }

  }else if(leftSonic.readDistance() != 0 && rightSonic.readDistance() == 0){
      state = 3;

      if (state != preState)
      {
        jump = 10;
        pid.prevSonicError = 0;
        pos_i_M1 = 0;
        pos_i_M2 = 0;
      }
      if (jump>0)
      {
        goStraightEnc(wallSpeed);
        jump = jump-1;
        rightCount++;

        if(rightCount >= 5){
          foundRight = true;
        }
      }
      else
      {
        int err = sideGapLeft - leftSonic.readDistance();
        int correction = pid.getSonicCorrection(err);
        mdriver.applySonicPid(correction);
      }

  }else if(leftSonic.readDistance() == 0 && rightSonic.readDistance() == 0){
      state = 4;

      if (state != preState)
      {
        jump = 10;
        pid.prevSonicError = 0;
        pos_i_M1 = 0;
        pos_i_M2 = 0;
      }
      if (jump>0)
      {
        goStraightEnc(wallSpeed);
        jump = jump-1;
        rightCount++;
        leftCount++;

        if(rightCount >= 5){
          foundRight = true;
        }
        if(leftCount >= 5){
          foundLeft = true;
        }

      }
      else
      {
        goStraightEnc(60);

      }
  }
  
  preState = state;
}

int wristDown = 170;
int wristUp = 80;
int wristBack = 15;

//t1
void grabCube(){
  driver.stop(500);
  float targetDist = 25;

  while((abs(driver.getLeftDistance() + driver.getRightDistance()) / 2.0) < targetDist){
    driver.runMotors(30.0, 30.0);
    driver.updateSpeed();
  }

  driver.stop(500);

  if(abs(tempLeftDist - tempRightDist) > 5){
    int diff = abs(tempLeftDist - tempRightDist);
    if(diff > 15){
      diff = 15;
    }

    if(tempLeftDist < tempRightDist){
      while(abs(driver.getLeftDistance()) < diff){
        driver.runMotors(0.0, -50.0);
        driver.updateSpeed();
      }
    }else{
      while(abs(driver.getRightDistance()) < diff){
        driver.runMotors(-50.0, 0.0);
        driver.updateSpeed();
      }
    }
  }

  targetDist = 60;

  while((abs(driver.getLeftDistance() + driver.getRightDistance()) / 2.0) < targetDist){
    driver.runMotors(-50.0, -50.0);
    driver.updateSpeed();
  }

  driver.stop(500);

  robot.setGripperWidth(90);
  robot.setWristAngle(wristDown);

  while(robot.isMoving()){
    robot.updateArm();
  }
  
  targetDist = 50;

  driver.stop(500);
  while((abs(driver.getLeftDistance() + driver.getRightDistance()) / 2.0) < targetDist){
    driver.updateSpeed();
    driver.runMotors(50.0, 50.0);

  }

  driver.stop(1000);

  robot.runGripperTo(30);
  //robot.runWristTo(95);

}

void placeCube1(){

  robot.runWristTo(wristDown);
  goStraightBackwardFor(50, 50);
  driver.stop(500);

  robot.runGripperTo(100);

  goStraightBackwardFor(20,50);
  driver.stop(500);

  robot.runWristTo(wristUp);

  turn180(80);
  driver.stop(500);

  goStraightBackwardFor(50, 50);

}

void placeCube2(){
  robot.runHeightTo(100);

  robot.runWristTo(wristBack);
  robot.runGripperTo(30);

  robot.runWristTo(wristUp);
  robot.runHeightTo(0);

  placeCube1();
  robot.setHeight(0);
}

char box2Colour = 'g';
void box1(){
  while(true){
    lineFollow();

    frontLox.read();
    if(frontLox.reading < 65 && frontLox.reading > 0){
      grabCube();
      break;
    }

    if(frontLox.reading < 160 && frontLox.reading > 0){
      approachingRamp = true;
    }

  }

  if(ldrSensor.identifyColor() == 'r'){
    box2Colour = 'g';
  }else{
    box2Colour = 'r';
  }
  Serial.println(box2Colour);
  
  driver.stop(1000);

  robot.runHeightTo(100);
  robot.runWristTo(wristBack);
  robot.runGripperTo(100);
  robot.runWristTo(wristUp);
  robot.runHeightTo(0);

  
}

void box2(){
  while(true){
    lineFollow();

    frontLox.read();
    if(frontLox.reading < 65 && frontLox.reading > 0){
      grabCube();
      break;
    }

    if(frontLox.reading < 160 && frontLox.reading > 0){
      approachingRamp = true;
    }
  }

  driver.stop(1000);

  robot.runWristTo(wristUp);

}

void goToTurn(char dir = 's'){
  driver.resetVars();
  while(true){
    keepAlive();
    qtr.read();
    if (qtr.pattern == 1) {
      int correction = pid.getLineCorrection(qtr.error);
      mdriver.applyLinePid(correction);

    }else{

      char pattern = qtr.pattern;

      if(pattern != 0){
        goStraightFor(130, 80);
        if(dir == 's'){break;}
        driver.stop(500);

        if (dir == 'l') {
          turnLeftTillMiddle();
        } else if (dir == 'r') {
          turnRightTillMiddle();
        }
        driver.stop(500);
        break;
        
      }
    }
  }
}

void goToPlaceBox(){
  driver.resetVars();
  while(true){
    keepAlive();
    qtr.read();
    if (qtr.pattern == 1) {
      int correction = pid.getLineCorrection(qtr.error);
      mdriver.applyLinePid(correction);

    }else{
      driver.stop(500);
      break;
      
    }
  }
}

void goToPlace(){
  while(!enteredCircle){
    lineFollow();

    if(frontLox.reading < 160 && frontLox.reading > 0){
      approachingRamp = true;
    }
  }
  
  char turn;
  if(box2Colour == 'r'){
    turn = 'l';
  }else{
    turn = 'r';

  }

  goToTurn(turn);
  goToPlaceBox();
  placeCube1();

  goToTurn('s');
  goToPlaceBox();
  placeCube2();

  goToTurn(turn);

}

void t1(){
  box1();

  box2();

  goToPlace();

  goToTurn('l');
  goToTurn('r');

  driver.stop(999999);

}

//t2
char postColours[3] = {'r', 'g', 'r'};
int postIndex = 0;

void goToJunc(char dir = 'l'){

  driver.resetVars();
  while(true){
    keepAlive();
    qtr.read();
    if (qtr.pattern == 1) {
      int correction = pid.getLineCorrectionVC(qtr.error);
      driver.applyLinePid(correction, 60);
      driver.updateSpeed();

    }else{

      char pattern = qtr.pattern;

      if(pattern != 0){
        goStraightFor(125, 60);
        driver.stop(500);

        if (dir == 'l') {
          turn90(60, 'l');
        } else if (dir == 'r') {
          turn90(60, 'r');
        }

        driver.stop(500);

        driver.resetVars();
        qtr.read();
        while(qtr.pattern == 0){
          qtr.read();
          goStraightBackward(60);
        }
        break;
      }
      
      float pushDistance = 55;
      goStraightFor(pushDistance, 60);

    }
    tcsSensor.setLedColor(0, 0, 0);
  }
}

void goToButtonPost(){
  int postSpeed = 60;

  driver.resetVars();
  while(true){
    keepAlive();
    qtr.read();
    if (qtr.pattern == 1) {
      int correction = pid.getLineCorrectionVC(qtr.error);
      driver.applyLinePid(correction, postSpeed);
      driver.updateSpeed();

    }else{
      int stopDist = 140;

      frontLox.read();
      if(frontLox.reading < stopDist && frontLox.reading > 0){  
        break;
      }
      float pushDistance = 60;
      goStraightFor(pushDistance, postSpeed);

      frontLox.read();
      if(frontLox.reading < stopDist && frontLox.reading > 0){  
        break;
      }
    
    }
  }
  driver.stop(500);
  // frontLox.read();
  
  // driver.kp = 0.9; // while sensing
  // driver.ki = 7.5;

  // while(frontLox.reading > 70){  
  //   keepAlive();
  //   goStraight(30);
  //   frontLox.read();
  // }

  // driver.kp = 0.7; // without sensing
  // driver.ki = 15;

  goStraightBackwardFor(40, postSpeed);
  driver.stop(500);

  robot.runWristTo(wristDown);
  robot.runGripperTo(80);

  goStraightFor(90,postSpeed);

  driver.stop(500);

}

void getButtonPostColour(){  

  goStraightBackwardFor(90, 60);
  driver.stop(1000);
  robot.runWristTo(wristUp);

  goStraightFor(60, 60);
  driver.stop(500);
  
  postColours[2] = tcsSensor.identifyColor();
  if(postColours[postIndex] == 'r'){
    tcsSensor.setLedColor(255, 0, 0);
  }else{
    tcsSensor.setLedColor(0, 255, 0);
  }
  postIndex++;
  
  goStraightBackwardFor(60,60);
  driver.stop(500);

  turn180(60);

  driver.stop(500);

  qtr.read();
  while(qtr.pattern == 0){
    qtr.read();
    goStraightBackward(60);
  }

}

void goToPost(){
  int postSpeed = 60;

  driver.resetVars();
  while(true){
    keepAlive();
    qtr.read();
    if (qtr.pattern == 1) {
      int correction = pid.getLineCorrectionVC(qtr.error);
      driver.applyLinePid(correction, postSpeed);
      driver.updateSpeed();

    }else{
      int stopDist = 140;

      frontLox.read();
      if(frontLox.reading < stopDist && frontLox.reading > 0){  
        break;
      }
      float pushDistance = 60;
      goStraightFor(pushDistance, postSpeed);

      frontLox.read();
      if(frontLox.reading < stopDist && frontLox.reading > 0){  
        break;
      }
    
    }
  }
  driver.stop(500);
  frontLox.read();
  
  // driver.kp = 0.9; // while sensing
  // driver.ki = 7.5;

  // while(frontLox.reading > 70){  
  //   keepAlive();
  //   goStraight(30);
  //   frontLox.read();
  // }

  // driver.kp = 0.7; // without sensing
  // driver.ki = 15;

  driver.stop(500);
}

void getPostColour(){
  postColours[postIndex] = tcsSensor.identifyColor();
  if(postColours[postIndex] == 'r'){
    tcsSensor.setLedColor(255, 0, 0);
  }else{
    tcsSensor.setLedColor(0, 255, 0);
  }
  postIndex++;

  goStraightBackwardFor(40, 60);
  driver.stop(500);

  turn180(60);

  driver.stop(500);

  qtr.read();
  while(qtr.pattern == 0){
    qtr.read();
    goStraightBackward(60);
  }

}

void t2(){

  robot.runGripperTo(80);
  robot.runHeightTo(22);
  robot.runWristTo(wristUp);

  goToJunc('l');
  goToPost();

  getPostColour();

  goToJunc('l');
  goToJunc('r');
  goToPost();

  getPostColour();

  goToJunc('r');
  goToJunc('l');

  goToButtonPost();

  getButtonPostColour();

  goToJunc('l');
  goToJunc('l');

  saveTowerCode(postColours);
}

//t3
int mazeHeight = 55;
int mazeTurningSpd = 50;

char enterMazeDir = 'r';
void wallFollowFor(float distanceMM){

  driver.resetVars();
  float targetDist = distanceMM;

  while((driver.getLeftDistance() + driver.getRightDistance()) / 2.0 < targetDist){
    keepAlive();
    wallFollow();
  }


}

void enterMaze(){
  while(!foundRight){
    wallFollow();
  }

  // wallFollowFor(55);
  // foundRight = false;

  // driver.stop(1000);
  
  // driver.resetVars();
  // // float targetDist1 = 149.225651046;
  // float targetDist =  402.699081699;

  // while(driver.getLeftDistance() < targetDist){
  //   keepAlive();
  //   driver.runMotors(50 * (141.371669412/402.699081699), 50);
  //   driver.updateSpeed();
  // }

  // driver.stop(1000);

  frontLox.read();
  while(true){
    wallFollow();
    frontLox.read();
    if(frontLox.reading <= 100 && frontLox.reading > 0){
      break;
    }
  }

  driver.stop(500);
  turn90(mazeTurningSpd, enterMazeDir);

  driver.stop(500);

  wallFollowFor(120);
  foundRight = false;
  foundLeft = false;


}

void enterRightCell(){
  while(!foundRight){
    wallFollow();
  }

  wallFollowFor(55);
  foundRight = false;


  driver.stop(1000);
  
  driver.resetVars();
  // float targetDist1 = 149.225651046;
  float targetDist =  402.699081699;

  while(driver.getLeftDistance() < targetDist){
    keepAlive();
    driver.runMotors(50 * (141.371669412/402.699081699), 50);
    driver.updateSpeed();
  }

  driver.stop(1000);

  while(true){
    wallFollow();
    if((l+r) > 300){break;}
  }

  wallFollowFor(60);
  
  driver.stop(500);

  frontLox.read();
  while(frontLox.reading >= 110 && frontLox.reading > 0){
    frontLox.read();
    wallFollow();
  }

  driver.stop(500);
  enterMazeDir = 'r';

}

void enterLeftCell(){
  while(!foundLeft){
    wallFollow();
  }

  wallFollowFor(55);
  foundLeft = false;


  driver.stop(1000);
  
  driver.resetVars();
  // float targetDist1 = 149.225651046;
  float targetDist =  402.699081699;

  while(driver.getRightDistance() < targetDist){
    keepAlive();
    driver.runMotors(50, 50 * (141.371669412/402.699081699));
    driver.updateSpeed();
  }

  driver.stop(1000);

  while(true){
    wallFollow();
    if((l+r) > 300){break;}
  }

  wallFollowFor(60);
  
  driver.stop(500);

  frontLox.read();
  while(frontLox.reading >= 110 && frontLox.reading > 0){
    frontLox.read();
    wallFollow();
  }

  driver.stop(500);
  enterMazeDir = 'l';

}

void exitMaze(){
  enterMazeDir = 'r';
  enterMaze();

  while(true){
    wallFollow();
    if(state != 1){
      break;
    }
  }

  driver.stop(500);
}

char ballColours[5] = {'r', 'g', 'r', 'g', 'r'};
char testColours[5] = {'r', 'g', 'r', 'r', 'g'};
int ballIndex = 0;

void getBall(){
  robot.runGripperTo(80);
  robot.runWristTo(wristDown);

  driver.resetVars();
  frontLox.read();
  while(frontLox.reading >= 85 && frontLox.reading > 0){
    frontLox.read();
    goStraight(50);
  }

  driver.stop(500);

  robot.runGripperTo(30);
  //ballColours[ballIndex] = ldrSensor.identifyColor();
  ballColours[ballIndex] = testColours[ballIndex];

  if(ballColours[ballIndex] == 'r'){
    feeder.requestStoreBall(RED);
  }else{
    feeder.requestStoreBall(GREEN);
  }

  ballIndex++;

  robot.setHeight(100);
  while(robot.isMoving()){
    robot.updateArm();
    feeder.updateFeeder();
  }

  robot.setWristAngle(wristBack);
  while(robot.isMoving()){
    robot.updateArm();
    feeder.updateFeeder();
  }

  robot.setGripperWidth(100);
  while(robot.isMoving()){
    robot.updateArm();
    feeder.updateFeeder();
  }

  robot.setWristAngle(wristUp);
  while(robot.isMoving()){
    robot.updateArm();
    feeder.updateFeeder();
  }

  robot.setHeight(mazeHeight);
  while(robot.isMoving()){
    robot.updateArm();
    feeder.updateFeeder();
  }
  // robot.runHeightTo(100);
  // robot.runWristTo(wristBack);
  // robot.runGripperTo(100);
  // robot.runWristTo(wristUp);
  // robot.runHeightTo(mazeHeight);

  driver.stop(1000);

}

void t3(){
  robot.runHeightTo(mazeHeight);
  robot.runWristTo(wristUp);

  enterMaze();

  enterRightCell();

  getBall();

  turn180(mazeTurningSpd, 'r');

  enterMaze();

  enterLeftCell();

  getBall();

  turn180(mazeTurningSpd, 'r');

  enterMaze();

  enterRightCell();

  getBall();

  turn180(mazeTurningSpd, 'r');

  enterMaze();

  enterLeftCell();

  getBall();

  turn180(mazeTurningSpd, 'r');

  enterMaze();

  enterRightCell();

  getBall();

  turn180(mazeTurningSpd, 'r');

  enterMaze();

  exitMaze();

  saveBallCode(ballColours);

}

//t4
void lineFollowAndTurn(char turn){
  while(true){
    keepAlive();
    float pushDis = 130;

    qtr.read();
    if (qtr.pattern == 1) {
      
      int correction = pid.getLineCorrection(qtr.error);
      mdriver.applyLinePid(correction);

    }else{
      driver.resetVars();
      float leftDist = driver.getLeftDistance() + pushDis;
      float rightDist = driver.getRightDistance() + pushDis;

      while(driver.getRightDistance() <= rightDist || driver.getRightDistance() <= leftDist){

        goStraight(80.0);

      }

      if(turn == 's'){
        break;
      }
      
      driver.stop(500);

      if (turn == 'l') {
        turnLeftTillMiddle();
      } else if (turn == 'r') {
        turnRightTillMiddle();
      }

      break;


    }
  }
  
}

void goToShoot(){
  lineFollowAndTurn('r');

  while(true){
    keepAlive();
    float pushDis = 130;

    qtr.read();
    if (qtr.pattern == 1) {
      
      int correction = pid.getLineCorrection(qtr.error);
      mdriver.applyLinePid(correction);

    }else{
      driver.resetVars();
      float leftDist = driver.getLeftDistance() + pushDis;
      float rightDist = driver.getRightDistance() + pushDis;

      while(driver.getRightDistance() <= rightDist || driver.getRightDistance() <= leftDist){

        goStraight(80.0);

      }

      if(turn == 's'){
        break;
      }
      
      driver.stop(500);

      if (turn == 'l') {
        turnLeftTillMiddle();
      } else if (turn == 'r') {
        turnRightTillMiddle();
      }

      break;


    }
  }

}

int greenBoxTarget = -1;
int redBoxTarget = -1;
void calculateThrowTargets(int* T, int* B) {
  
  // 1. Calculate the 4 elements of the 2x2 Matrix Result
  
  // R00 = (T0*T0) + (T1*T1) + (T2*T2) -> Which is just sum of T
  int r00 = T[0] + T[1] + T[2];

  // R11 = (B0*B0) + (B1*B1) + (B2*B2) -> Which is just sum of B
  int r11 = B[0] + B[1] + B[2];

  // R01 and R10 are identical: Sum of products (Dot Product)
  // (T0*B0) + (T1*B1) + (T2*B2)
  int crossSum = (T[0] * B[0]) + (T[1] * B[1]) + (T[2] * B[2]);
  
  int r01 = crossSum;
  int r10 = crossSum;

  // 2. Summation of all 4 elements
  int totalSum = r00 + r01 + r10 + r11;
  
  Serial.print("Matrix Sum: ");
  Serial.println(totalSum);

  // 3. Calculate Quotient and Remainder
  greenBoxTarget = totalSum / 4; // Integer Division (Quotient)
  redBoxTarget   = totalSum % 4; // Modulo (Remainder)
}

int currentAngle = 0;
const float MM_PER_DEG = 2.22;
void aimAndUpdate(int boxID) {
  
  int targetAngle = 0;

  // 1. Get the Absolute Angle for the target box
  switch(boxID) {
    case 0: targetAngle = 135;  break; // Box A
    case 1: targetAngle = -135; break; // Box B
    case 2: targetAngle = 45;   break; // Box C
    case 3: targetAngle = -45;  break; // Box D
  }

  Serial.print("Moving from ");
  Serial.print(currentAngle);
  Serial.print(" to Box ID ");
  Serial.println(boxID);

  // 2. Calculate Difference
  int diff = targetAngle - currentAngle;

  // 3. Normalize to shortest path (-180 to 180)
  // This handles the "wrap around" (e.g. going from 135 to -135 is just 90 deg right)
  while (diff > 180)  diff -= 360;
  while (diff <= -180) diff += 360;

  // 4. Execute Turn
  float moveDist = abs(diff) * MM_PER_DEG;

  if (diff > 0) {
    Serial.print("Turning RIGHT degrees: ");
    Serial.println(diff);
    turnRightFor(moveDist);
  } else if (diff < 0) {
    Serial.print("Turning LEFT degrees: ");
    Serial.println(abs(diff));
    turnLeftFor(moveDist);
  } else {
    Serial.println("Already at target angle.");
  }

  driver.stop(500);

  // 5. Update system state
  currentAngle = targetAngle;
}

void t4(){
  
}

void setup() {
  Serial.begin(9600);

  ldrSensor.begin();
  tcsSensor.begin();

  // Assign EEPROM Addresses
  ldrSensor.setEEPROMAddress(EEPROM_ADDR_LDR);
  tcsSensor.setEEPROMAddress(EEPROM_ADDR_TCS);

  // Motor Encoder Setup
  pinMode(M1_ENCODER_A, INPUT);
  pinMode(M1_ENCODER_B, INPUT);
  pinMode(M2_ENCODER_A, INPUT);
  pinMode(M2_ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A), readEncoderM1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENCODER_A), readEncoderM2, RISING);

  // Lift Encoder Setup
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoderLift, RISING);
  
  robot.begin();
  feeder.begin();

  frontLox.init();

  forceCalibrate = (digitalRead(CALIBRATE_BTN_PIN) == LOW);
  
  //manualCalibrationArm();
  switchCalibrationArm();
  qtr.calibrate(5);

  robot.setHome(); 
  driver.resetVars();

  //calibrateColourSensors();

  // frontLox.read();
  // while(frontLox.reading > 80 || frontLox.reading == -1){frontLox.read();}

}

void loop() {

} 


