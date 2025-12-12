#ifndef FEEDER_H
#define FEEDER_H

#include <Arduino.h>
#include <Stepper.h>
#include <Servo.h>

enum BallColor { EMPTY = 0, RED = 1, GREEN = 2 };
enum FeederState { IDLE, MOVING, WAITING_DROP, SHOOTING_OPEN, SHOOTING_FIRE, SHOOTING_RESET };

class Feeder {
  private:
    // --- Hardware Objects ---
    Stepper* stepper;
    Servo doorServo;
    uint8_t gunTriggerPin;

    // --- Motor Control Variables ---
    long currentStepCount;     // Tracks absolute position (virtual)
    long targetStepCount;      // Where we want to be
    unsigned long lastStepTime;
    int stepDelay;             // Delay between steps in micros (control speed manually)
    
    // --- Servo & Gun Variables ---
    unsigned long stateTimer;  // Generic timer for delays (ball drop, servo wait)
    FeederState currentState;  // What are we doing right now?
    FeederState nextState;     // What to do after moving?

    // --- Logic Variables ---
    BallColor slots[5];
    int currentSlotAtInput;    // Which physical slot index (0-4) is at Input Hole?
    int stepsPerRev;
    int stepsPerSlot;
    int outputOffset;
    
    // Temporary variables to remember what to do after a move
    BallColor pendingColorAdd;    // Color to add after move finishes
    int pendingSlotClear;         // Slot to clear after shooting finishes
    BallColor shootingColorTarget;// Color we are currently trying to shoot all of

    // --- Private Logic ---
    void runMotorLogic();
    void runStateLogic();
    
    int findBestSlotFor(BallColor color);
    int findNearestSlotWithColor(BallColor color);
    long calculateShortestPath(int targetSlot);

  public:
    // --- CONSTRUCTOR ---
    Feeder(uint8_t* stepperPins, int stepsPerRev, 
           uint8_t servoPin, uint8_t gunPin);

    void begin();

    // --- Non-Blocking Requests ---
    // Returns true if request accepted, false if busy or full
    bool requestStoreBall(BallColor color);
    bool requestShootAll(BallColor color);

    // --- The Main Worker ---
    void updateFeeder(); // Call this frequently in loop()
    
    // --- Utilities ---
    bool isBusy();
    int getCount(BallColor color);
};

#endif