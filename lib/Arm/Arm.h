#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include <Stepper.h>

class Arm {
  private:
    // --- Gripper Variables ---
    float stepsPerMM_Grip = 0;
    float maxGripperWidth = 0;

    Stepper* stepper;
    long currentGripperSteps;   
    long targetGripperSteps;    
    bool gripperActive;         
    
    // --- Vertical Motor Variables ---
    float pulsesPerMM_Vert = 1.93;  
    int motorSpeedMax = 200;
    int motorSpeedMin = 60;

    uint8_t pinMotorPWM;
    uint8_t pinMotorDir1;
    uint8_t pinMotorDir2;
    
    volatile long* encoderPos;
    
    long currentVerticalSteps;   
    long targetVerticalPulses;  
    bool verticalActive;        

    void runVerticalLogic();
    void runGripperLogic();

  public:
    // --- CONSTRUCTOR ---
    Arm(uint8_t* stepperPins, int stepsPerRev,
        uint8_t* dcMotorPins, volatile long* externalEncoderCount);

    void begin();
    void setMotorSpeed(int maxSpd, int minSpd);
    void setHome(); 

    // --- Manual Calibration (New) ---
    // Moves motors for a specific duration (ms)
    // Positive time = Close/Up, Negative time = Open/Down
    void manualGripper(int timeMs);
    void manualVertical(int timeMs);

    // --- Non-Blocking Setters ---
    void setGripperWidth(float widthMM); 
    void moveToHeight(float heightMM);
    
    // --- Blocking Movement ---
    void runTo(float heightMM, float widthMM);
    void runHeightTo(float heightMM); 
    void runGripperTo(float widthMM); 
    
    // --- The Main Worker ---
    void updateArm();      
    bool isMoving();    
    
    // Utilities
    float getCurrentHeight();
    float getCurrentWidth();
};

#endif