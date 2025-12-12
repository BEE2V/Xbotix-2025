#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include <Stepper.h>
#include <Servo.h>

class Arm {
  private:
    // --- Gripper Variables ---
    float stepsPerMM_Grip = 15;
    float maxGripperWidth = 90;
    float minGripperWidth = 34;

    Stepper* stepper;
    long currentGripperSteps;   
    long targetGripperSteps;    
    bool gripperActive;         
    
    // --- Vertical Motor Variables ---
    float pulsesPerMM_Vert = 1.93;  
    int motorSpeedMax = 255;
    int motorSpeedMin = 150;

    uint8_t pinMotorPWM;
    uint8_t pinMotorDir1;
    uint8_t pinMotorDir2;
    
    volatile long* encoderPos;
    
    long currentVerticalSteps;   
    long targetVerticalPulses;  
    bool verticalActive;
    
    // --- Wrist Servo Variables ---
    uint8_t homeAngle = 160;
    uint8_t topAngle = 80;

    uint8_t wristAngleMin = 0;
    uint8_t wristAngleMax = 160;
    uint16_t angleDelay = 25; // ms per degree

    Servo wristServo;
    uint8_t pinServo;
    int currentWristAngle;
    int targetWristAngle;
    volatile long lastWristUpdate;
    bool wristActive;

    void runVerticalLogic();
    void runGripperLogic();
    void runWristLogic();

  public:
    // --- CONSTRUCTOR ---
    Arm(uint8_t* stepperPins, int stepsPerRev,
        uint8_t* dcMotorPins, volatile long* externalEncoderCount,
        uint8_t servoPin);

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
    void setHeight(float heightMM);
    void setWristAngle(float angleDeg);
    
    // --- Blocking Movement ---
    void runTo(float heightMM, float widthMM);
    void runHeightTo(float heightMM); 
    void runGripperTo(float widthMM); 
    void runWristTo(int angleDeg);
    
    // --- The Main Worker ---
    void updateArm();      
    bool isMoving();    
    
    // Utilities
    float getCurrentHeight();
    float getCurrentWidth();
};

#endif