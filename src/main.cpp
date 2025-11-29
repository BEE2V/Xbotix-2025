#include <Arduino.h>
#include "Arm.h"

// --- CONFIGURATION ---
const int STEPS_PER_REV = 2048;

// --- Encoder Pins ---
#define ENC_A       18 // Interrupt Pin
#define ENC_B       46 // Digital Pin

volatile long encoderCount = 0;

// --- INITIALIZATION ---
Arm robot(
  // Stepper: IN1, IN2, IN3, IN4
  const_cast<uint8_t *>((const uint8_t[]) {53, 51, 52, 50}), 
  STEPS_PER_REV, 
  // DC Motor: PWM, IN1, IN2
  const_cast<uint8_t *>((const uint8_t[]) {13, 16, 17}), 
  &encoderCount
);

// --- Interrupt Service Routine ---
void updateEncoder() {
  if (digitalRead(ENC_B) == LOW) {
    encoderCount--;
  } else {
    encoderCount++;
  }
}

void setup() {
  Serial.begin(9600);

  // Encoder Setup
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), updateEncoder, RISING);
  
  robot.begin();
  
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

  // --- CALIBRATION DONE ---
  // The current physical position is now "Zero"
  Serial.println("Setting Home Position...");
  robot.setHome(); 
  Serial.println("System Ready.");
}

void loop() {
  // Main logic starts here after calibration
  Serial.println("Main Loop Running...");
  delay(2000);
  
  // // Example: Move somewhere
  robot.runHeightTo(10.0);
  delay(99999);
  // delay(1000);
  // robot.runHeightTo(0.0);
  // delay(2000);
}


