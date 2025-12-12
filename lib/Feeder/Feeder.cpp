#include "Feeder.h"

Feeder::Feeder(uint8_t* stepperPins, int stepsRev, uint8_t servoPin, uint8_t gunPin) {
  // Standard Stepper Init
  // Note: For 28BYJ-48, standard Stepper lib works best if you swap pins 2 & 3 in the constructor
  // to match the coil sequence (1-3-2-4)
  stepper = new Stepper(stepsRev, stepperPins[0], stepperPins[2], stepperPins[1], stepperPins[3]);
  
  // doorServo.attach(servoPin);
  gunTriggerPin = gunPin;
  
  stepsPerRev = stepsRev;
  stepsPerSlot = stepsPerRev;
  
  currentStepCount = 0;
  targetStepCount = 0;
  currentSlotAtInput = 0;
  
  // Set speed (Lower delay = Higher speed)
  // 28BYJ-48 max speed is around 1000 micros delay
  stepDelay = 2000; 

  currentState = IDLE;
  
  for(int i=0; i<5; i++) slots[i] = EMPTY;
}

void Feeder::begin() {
  pinMode(gunTriggerPin, OUTPUT);
  digitalWrite(gunTriggerPin, HIGH);
  doorServo.write(0); // Close Door

  stepper->setSpeed(15);
}

// --- MAIN UPDATE LOOP ---
void Feeder::updateFeeder() {
  runMotorLogic();  // Handles physical movement step-by-step
  runStateLogic();  // Handles sequence (Move -> Open Door -> Shoot)
}

// --- LOGIC: Handles the "Brain" ---
void Feeder::runStateLogic() {
  unsigned long now = millis();

  switch (currentState) {
    
    case IDLE:
      // Do nothing, waiting for command
      break;

    case MOVING:
      // Check if motor reached target
      if (currentStepCount == targetStepCount) {
        currentState = nextState; // Transition to next phase
        stateTimer = now;         // Start timer for delays
      }
      break;

    case WAITING_DROP:
      // We arrived at input hole. Wait for ball to fall in.
      if (now - stateTimer > 800) { // 800ms delay
        // Logic: Ball is now in the slot
        slots[currentSlotAtInput] = pendingColorAdd;
        currentState = IDLE;
      }
      break;

    case SHOOTING_OPEN:
      // We arrived at Output hole. Open door.
      doorServo.write(90); 
      if (now - stateTimer > 300) { // Wait for servo
        // Fire gun
        digitalWrite(gunTriggerPin, HIGH);
        stateTimer = now;
        currentState = SHOOTING_FIRE;
      }
      break;

    case SHOOTING_FIRE:
      // Gun trigger is held HIGH
      if (now - stateTimer > 150) { // 150ms trigger pull
        digitalWrite(gunTriggerPin, LOW);
        stateTimer = now;
        currentState = SHOOTING_RESET;
      }
      break;

    case SHOOTING_RESET:
      // Wait for ball to leave, then close door
      if (now - stateTimer > 500) { 
        doorServo.write(0); // Close
        slots[pendingSlotClear] = EMPTY; // Clear memory
        
        // Check if we need to shoot more of the same color
        int nextSlot = findNearestSlotWithColor(shootingColorTarget);
        if (nextSlot != -1) {
          // More balls exist! Setup next shot immediately
          long steps = calculateShortestPath(nextSlot); // To Input
          steps += outputOffset; // To Output
          
          targetStepCount = currentStepCount + steps;
          
          // Logic: Update tracking for where input hole is now
          int slotsMoved = steps / stepsPerSlot;
          currentSlotAtInput = (currentSlotAtInput - slotsMoved) % 5;
          if (currentSlotAtInput < 0) currentSlotAtInput += 5;
          
          pendingSlotClear = nextSlot;
          currentState = MOVING;
          nextState = SHOOTING_OPEN;
        } else {
          // No more balls
          currentState = IDLE;
        }
      }
      break;
  }
}

// --- LOGIC: Handles the Stepper (Non-Blocking) ---
void Feeder::runMotorLogic() {
  if (currentStepCount == targetStepCount){
    return;
  } // Not moving

  unsigned long now = micros();
  if (now - lastStepTime >= stepDelay) {
    lastStepTime = now;
    
    if (targetStepCount > currentStepCount) {
      stepper->step(1);
      currentStepCount++;
    } else {
      stepper->step(-1);
      currentStepCount--;
    }
  }
}


bool Feeder::requestStoreBall(BallColor color) {
  if (currentState != IDLE) return false; // Busy

  int bestSlot = findBestSlotFor(color);
  if (bestSlot == -1) return false; // Full

  // Setup Move
  long steps = calculateShortestPath(bestSlot);
  
  targetStepCount = currentStepCount + steps;
  currentSlotAtInput = bestSlot; // We will be at this slot when move ends
  
  pendingColorAdd = color;
  currentState = MOVING;
  nextState = WAITING_DROP;
  
  return true;
}

bool Feeder::requestShootAll(BallColor color) {
  if (currentState != IDLE) return false;

  int firstSlot = findNearestSlotWithColor(color);
  if (firstSlot == -1) return false; // None found

  // Setup Move to Output
  long stepsToIn = calculateShortestPath(firstSlot);
  long totalSteps = stepsToIn + outputOffset;

  targetStepCount = currentStepCount + totalSteps;
  
  // Logic update
  int slotsMoved = totalSteps / stepsPerSlot;
  currentSlotAtInput = (currentSlotAtInput - slotsMoved) % 5;
  if (currentSlotAtInput < 0) currentSlotAtInput += 5;

  shootingColorTarget = color; // Remember what we are shooting
  pendingSlotClear = firstSlot;
  
  currentState = MOVING;
  nextState = SHOOTING_OPEN;
  
  return true;
}

// --- UTILS ---

long Feeder::calculateShortestPath(int targetSlot) {
  int diff = targetSlot - currentSlotAtInput;
  if (diff > 2) diff -= 5;
  if (diff < -2) diff += 5;
  return diff * stepsPerSlot;
}

int Feeder::findBestSlotFor(BallColor color) {
  int bestSlot = -1;
  int maxScore = -100;

  for (int i = 0; i < 5; i++) {
    if (slots[i] == EMPTY) {
      int score = 0;
      int left = (i == 0) ? 4 : i - 1;
      int right = (i == 4) ? 0 : i + 1;

      if (slots[left] == color) score += 10;
      if (slots[right] == color) score += 10;
      if (slots[left] != EMPTY && slots[left] != color) score -= 5;
      if (slots[right] != EMPTY && slots[right] != color) score -= 5;
      
      int dist = abs(currentSlotAtInput - i);
      if (dist > 2) dist = 5 - dist;
      score -= dist;

      if (score > maxScore) {
        maxScore = score;
        bestSlot = i;
      }
    }
  }
  return bestSlot;
}

int Feeder::findNearestSlotWithColor(BallColor color) {
  int bestSlot = -1;
  int shortestDist = 100;
  for (int i = 0; i < 5; i++) {
    if (slots[i] == color) {
      int dist = abs(currentSlotAtInput - i);
      if (dist > 2) dist = 5 - dist;
      if (dist < shortestDist) { shortestDist = dist; bestSlot = i; }
    }
  }
  return bestSlot;
}

bool Feeder::isBusy() {
  return (currentState != IDLE);
}

int Feeder::getCount(BallColor color) {
  int c = 0;
  for (int i = 0; i < 5; i++) if (slots[i] == color) c++;
  return c;
}